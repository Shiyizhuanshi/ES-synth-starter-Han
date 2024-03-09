#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <bitset>
#include <ES_CAN.h>
#include <cmath>

#include "read_inputs.h"
#include "pin_definitions.h"
#include "read_inputs.h"


//Constants
const uint32_t interval = 100; //Display update interval

uint32_t ID = 0x123; //CAN ID
uint8_t RX_Message[8] = {0};  //CAN RX message
volatile uint8_t TX_Message[8] = {0}; //CAN TX message

//Create message input and output queues
//36 messages of 8 bytes, each message takes around 0.7ms to process
QueueHandle_t msgInQ = xQueueCreate(36,8);; // Message input queue
QueueHandle_t msgOutQ = xQueueCreate(36,8);; // Message output queue

SemaphoreHandle_t CAN_TX_Semaphore; //CAN TX semaphore

//Create task handles
TaskHandle_t scanKeysHandle = NULL;
TaskHandle_t displayUpdateHandle = NULL;
TaskHandle_t decodeTaskHandle = NULL;
TaskHandle_t CAN_TX_Handle = NULL;

//Struct to hold system state
struct {
  std::bitset<28> inputs;
  SemaphoreHandle_t mutex;  
  std::array<knob, 4> knobValues;
  uint32_t local_boardId = HAL_GetUIDw0();
  int posId = 0;
} sysState;

volatile uint32_t currentStepSize;

const uint32_t sampleRate = 22000;  //Sample rate

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

void initial_display(){
    setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
    delayMicroseconds(2);
    setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply
}

void sampleISR() {
  if (sysState.posId == 0 ){
    static uint32_t phaseAcc = 0;
    phaseAcc = currentStepSize ? phaseAcc + currentStepSize : 0;
    int32_t Vout = (phaseAcc >> 24) - 128;
    Vout = Vout >> (8 - sysState.knobValues[3].current_knob_value);
    analogWrite(OUTR_PIN, Vout + 128);
  }
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void scanKeysTask(void * pvParameters) {

  
  volatile uint32_t localCurrentStepSize1;

  const TickType_t xFrequency1 = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime1 = xTaskGetTickCount();
  std::bitset<12> keys;
  std::bitset<8> current_knobs;
  std::bitset<12> previou_keys("111111111111");
  std::bitset<8> previous_knobs("00000000");
  std::bitset<1> WestDetect;
  std::bitset<1> EastDetect;

  while (1){ 
    vTaskDelayUntil( &xLastWakeTime1, xFrequency1);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = readInputs();

    keys = extractBits<28, 12>(sysState.inputs, 0, 12);
    current_knobs = extractBits<28, 8>(sysState.inputs, 12, 8);
    WestDetect = extractBits<28, 1>(sysState.inputs, 23, 1);
    EastDetect = extractBits<28, 1>(sysState.inputs, 27, 1);
    updateKnob(sysState.knobValues, previous_knobs, current_knobs);

    if (EastDetect[0] && WestDetect[0]){
      sysState.posId = 0;
      for (int i = 0; i < 12; i++){
        if (keys.to_ulong() != 0xFFF){
          if (!keys[i]) {
          localCurrentStepSize1 = stepSizes[i];
          }
        }
        else{
          localCurrentStepSize1 = 0;
        }
      }
      // Serial.println(localCurrentStepSize1);
      __atomic_store_n(&currentStepSize, localCurrentStepSize1, __ATOMIC_RELAXED);
    }
    else{
      for (int i = 0; i < 12; i++){
        if (keys[i] != previou_keys[i]){
          TX_Message[0] = keys[i] ? 'R' : 'P';
          TX_Message[1] = i;
          TX_Message[2] = sysState.posId + 3;
          TX_Message[3] = sysState.posId;
          // CAN_TX(0x123, const_cast<uint8_t*>(TX_Message));
          xQueueSend( msgOutQ, const_cast<uint8_t*>(TX_Message), portMAX_DELAY);
        }
      }
    }

    previou_keys = keys;
    previous_knobs = current_knobs;
    xSemaphoreGive(sysState.mutex);
  }
}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency2 = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime2 = xTaskGetTickCount();
  static uint32_t count = 0;

  while (1) {
    vTaskDelayUntil( &xLastWakeTime2, xFrequency2);
    u8g2.clearBuffer();

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    
    //Display inputs
    for (int i = 0; i < 20; i++){
      u8g2.setCursor(5*(i+1), 10);
      u8g2.print(sysState.inputs[i]);
    }
    xSemaphoreGive(sysState.mutex);

    //Display knobs
    for (int i = 0; i < 4; i++){
      u8g2.setCursor(10*(i+1), 20);
      u8g2.print(sysState.knobValues[i].current_knob_value);
    }

    for (int i = 0; i < 4; i++){
      if (i == sysState.posId){
        u8g2.drawBox(10*(i+1), 23, 7, 7);
      }
      else{
        u8g2.drawFrame(10*(i+1), 23, 7, 7);
      }
    }
    
    u8g2.setCursor(66,30);
    u8g2.print((char) RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);
 
    u8g2.sendBuffer();
	  
    digitalToggle(LED_BUILTIN);
  }
}

void decodeTask(void * pvParameters) {
  std::bitset<12> keys_1;
  std::bitset<12> previou_keys_1("111111111111");
  volatile uint32_t localCurrentStepSize2;
  while (1) {
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    if (sysState.posId == 0){
      if (RX_Message[0] == 'P'){
        sysState.inputs[RX_Message[1]] = 0;
      }
      else if (RX_Message[0] == 'R'){
        sysState.inputs[RX_Message[1]] = 1;
      }
      keys_1 = extractBits<28, 12>(sysState.inputs, 0, 12);
      for (int i = 0; i < 12; i++){
        if (keys_1.to_ulong() != 0xFFF){
          if (keys_1[i] != previou_keys_1[i]){
            localCurrentStepSize2 = !keys_1[i] ? stepSizes[i] : 0;
          }
        }
        else{
          localCurrentStepSize2 = 0;
        }
      }
      previou_keys_1 = keys_1;
      localCurrentStepSize2 = localCurrentStepSize2 * pow(2, RX_Message[2] - 4);
      // Serial.println(localCurrentStepSize2);
      __atomic_store_n(&currentStepSize, localCurrentStepSize2, __ATOMIC_RELAXED);
    }
    // // if board 1 receives a message from board 0, it will send the message back to the board 0
    else if(RX_Message[3] == 0 && sysState.posId == 1
          && (RX_Message[0] == 'R' || RX_Message[0] == 'P')){
        RX_Message[3] = 1;
        xQueueSend( msgOutQ, const_cast<uint8_t*>(RX_Message), portMAX_DELAY);
    }
    
    Serial.print("RX: ");
    Serial.print((char) RX_Message[0]);
    Serial.print(RX_Message[1]);
    Serial.print(RX_Message[2]);
    Serial.println();

  }
}

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    Serial.print("TX: ");
    Serial.print((char) msgOut[0]);
    Serial.print(msgOut[1]);
    Serial.print(msgOut[2]);
    Serial.println();
		CAN_TX(ID, msgOut);
    
	}
}

void send_handshake_signal(int stateW, int stateE){
    setRow(5);
    delayMicroseconds(3);
    digitalWrite(OUT_PIN, stateW);
    delayMicroseconds(3);
    setRow(6);
    delayMicroseconds(3);
    digitalWrite(OUT_PIN, stateE);
    delayMicroseconds(3);
}

  // reutrn the posId of the board
  int auto_detect_init(){
  uint8_t detext_RX_Message[8] = {0};  //CAN RX message
  uint8_t detext_TX_Message[8] = {0};  //CAN RX message
  uint32_t detect_CAN_ID = 0x123; //CAN ID
    int wait_count = 300;
    int ready_board_count = 0;
    std::bitset<28> inputs;
    std::bitset<1> WestDetect;
    std::bitset<1> EastDetect;
    
    // // send and receive handshake signals for 3s
    // while (wait_count > 0){
    //     wait_count--;
    //     send_handshake_signal(1,1);
    //     delay(10);
    //     inputs = readInputs();
    // }
    for (int i = 0; i < 100; i++){
        send_handshake_signal(1,1);
        delay(30);
    }
    inputs = readInputs();
    WestDetect = extractBits<28, 1>(inputs, 23, 1);
    EastDetect = extractBits<28, 1>(inputs, 27, 1);
    delay(2000);
    //after 3s, sends handshake status to the one board to process
    //if the board is the most west then it is the main board which needs to 
    //process the handshake status of the other boards
    //if not then it is the other boards which need to send the handshake status to the main board
    if (!WestDetect[0]){
        Serial.println("west borad detected");
        // if west detect, waiting for message from the main board
        do{
            Serial.println("waiting for main board message!");
            delay(100);
            while (CAN_CheckRXLevel())
	          CAN_RX(detect_CAN_ID, detext_RX_Message);
            Serial.println("main board message received!");
            Serial.print((char)detext_RX_Message[0]);
            Serial.println(detext_RX_Message[1]);
        } while (detext_RX_Message[0] !=  'C' && detext_RX_Message[1] != 0);
        // when main board is determined, set all handshake signals to 0
        // give time for all boards to receive the handshake signal
        for (int i = 0; i < 10; i++){
            send_handshake_signal(0,0);
            delay(30);
        }
        // keep updating the handshake signal until the west is detected
        do{
            inputs = readInputs();
            WestDetect = extractBits<28, 1>(inputs, 23, 1);
            Serial.println(WestDetect[0]);
            delay(30);
        } while (WestDetect[0]);
        delay(500);
        Serial.println("update west detect!");
        //once west is detected, wait the message from west board
        do{
            Serial.println("waiting for west board message!");
            while (CAN_CheckRXLevel()) CAN_RX(detect_CAN_ID, detext_RX_Message);
            Serial.println("west borad message received!");
            Serial.print((char)detext_RX_Message[0]);
            Serial.println(detext_RX_Message[1]);
        } while (detext_RX_Message[0] != 'M');

        // CAN_RX(detect_CAN_ID, detext_RX_Message);
        // Serial.println("west borad message received!");
        // Serial.print((char)detext_RX_Message[0]);
        // Serial.println(detext_RX_Message[1]);

        for (int i = 0; i < 10; i++){
            send_handshake_signal(1,1);
            delay(30);
        }
        Serial.println("update east detect!");
        detext_TX_Message[0] = 'M';
        detext_TX_Message[1] = detext_RX_Message[1] + 1;
        //send board position to next board
        if (detext_RX_Message[1] != 2){
          CAN_TX(detect_CAN_ID, detext_TX_Message);
          Serial.println("send board pos to east board");
          Serial.print((char) detext_TX_Message[0]);
          Serial.println(detext_TX_Message[1]);
        }
        else{
          Serial.println("I am the most east board!");
          Serial.println(detext_TX_Message[1]);
        }
        return detext_TX_Message[1];
    }
    else{
        delay(1000);
        Serial.println("I am the main board!");
        // if nothing on the west, then it is the main board with position 0
        detext_TX_Message[0] = 'C';
        detext_TX_Message[1] = 0;
        // tell the other boards that the main board is determined
        CAN_TX(detect_CAN_ID, detext_TX_Message);
        Serial.println("send main board message to other boards!");
        //give the other boards time to receive the message
        delay(100);
        send_handshake_signal(1,1);
        //give the other boards time to update West Detect
        delay(100);
        detext_TX_Message[0] = 'M';
        CAN_TX(detect_CAN_ID, detext_TX_Message);
        Serial.println("send board info to other boards!");
        Serial.print((char) detext_TX_Message[0]);
        Serial.println(detext_TX_Message[1]);
        return 0;
    }
}

void setup() {
  sysState.knobValues[2].current_knob_value = 4;
  sysState.knobValues[3].current_knob_value = 6;
  //Set pin directions
  set_pin_directions();

  initial_display();

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  //Initialise sample timer
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise CAN TX semaphore
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  //Initialise CAN Bus
  CAN_Init(false);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();

  //Initialise serial port
  Serial.begin(9600);
  Serial.println("Serial port initialised");
  
  sysState.posId = auto_detect_init();
  initial_display();
  Serial.print("posId: ");
  Serial.println(sysState.posId);
  sysState.knobValues[2].current_knob_value = sysState.posId + 3;

  //Create tasks
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		/* Text name for the task */
  256 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &displayUpdateHandle );	/* Pointer to store the task handle */

  xTaskCreate(
  decodeTask,		/* Function that implements the task */
  "decode",		/* Text name for the task */
  256 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdateHandle );	/* Pointer to store the task handle */

  xTaskCreate(
  CAN_TX_Task,		/* Function that implements the task */
  "CAN_TX",		/* Text name for the task */
  256 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &CAN_TX_Handle );	/* Pointer to store the task handle */
  
  sysState.mutex = xSemaphoreCreateMutex(); //Create mutex
  vTaskStartScheduler();
}

void loop() {
}