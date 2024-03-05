#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <bitset>
#include <ES_CAN.h>
#include <cmath>

#include "read_inputs.h"
#include "pin_definitions.h"

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

//Struct to hold system state
struct {
  std::bitset<20> inputs;
  SemaphoreHandle_t mutex;  
  std::array<knob, 4> knobValues;
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

void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc = currentStepSize ? phaseAcc + currentStepSize : 0;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - sysState.knobValues[3].current_knob_value);
  analogWrite(OUTR_PIN, Vout + 128);
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void scanKeysTask(void * pvParameters) {

  
  // volatile uint32_t localCurrentStepSize;

  const TickType_t xFrequency1 = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime1 = xTaskGetTickCount();
  std::bitset<12> keys;
  std::bitset<8> current_knobs;
  std::bitset<12> previou_keys("111111111111");
  std::bitset<8> previous_knobs("00000000");
  while (1){ 
    vTaskDelayUntil( &xLastWakeTime1, xFrequency1);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = readInputs();

    keys = extractBits<20, 12>(sysState.inputs, 0, 12);
    current_knobs = extractBits<20, 8>(sysState.inputs, 12, 8);
    
    updateKnob(sysState.knobValues, previous_knobs, current_knobs);

    xSemaphoreGive(sysState.mutex);

    for (int i = 0; i < 12; i++){
      // if (keys.to_ulong() != 0xFFF){
      //   if (!keys[i]) {
      //   localCurrentStepSize = stepSizes[i];
      //   }
      // }
      // else{
      //   localCurrentStepSize = 0;
      // }
      
      // Decode keys
      if (keys[i] != previou_keys[i]){
        TX_Message[0] = keys[i] ? 'R' : 'P';
        TX_Message[1] = i;
        TX_Message[2] = 4;
        // CAN_TX(0x123, const_cast<uint8_t*>(TX_Message));
        xQueueSend( msgOutQ, const_cast<uint8_t*>(TX_Message), portMAX_DELAY);
      }
    }

    previou_keys = keys;
    previous_knobs = current_knobs;
    // __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency2 = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime2 = xTaskGetTickCount();
  static uint32_t count = 0;
  while (1) {
    vTaskDelayUntil( &xLastWakeTime2, xFrequency2);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    
    //Display inputs
    // for (int i = 0; i < 20; i++){
    //   u8g2.setCursor(5*(i+1), 10);
    //   u8g2.print(sysState.inputs[i]);
    // }
    u8g2.setCursor(10, 10);
    u8g2.print("Position: ");

    u8g2.drawFrame(70, 2, 8, 8);
    u8g2.drawFrame(80, 2, 8, 8);
    u8g2.drawFrame(90, 2, 8, 8);
    u8g2.drawBox(100, 2, 8, 8);

    //Display knobs
    for (int i = 0; i < 4; i++){
      u8g2.setCursor(90 + 8*(i+1), 30);
      u8g2.print(sysState.knobValues[i].current_knob_value);
    }
    std::vector<std::string> pressedKeys;
    for (int i = 0; i < 12; i++){
      if (sysState.inputs[i] == 0){
        pressedKeys.push_back(noteNames[i]);
      }
    }

    for (int i = 0; i < pressedKeys.size(); i++){
      u8g2.setCursor(10+ 10*i, 20);
      u8g2.print(pressedKeys[i].c_str());
    }

    xSemaphoreGive(sysState.mutex);
    // u8g2.setCursor(66,30);
    // u8g2.print((char) RX_Message[0]);
    // u8g2.print(RX_Message[1]);
    // u8g2.print(RX_Message[2]);

    u8g2.sendBuffer();
	  
    digitalToggle(LED_BUILTIN);
  }
}

void decodeTask(void * pvParameters) {
  std::bitset<12> keys_1;
  std::bitset<12> previou_keys_1("111111111111");
  volatile uint32_t localCurrentStepSize = 0;
  while (1) {
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    Serial.print((char) RX_Message[0]);
    Serial.print(RX_Message[1]);
    Serial.print(RX_Message[2]);
    Serial.println();

    if (RX_Message[0] == 'P'){
      localCurrentStepSize += stepSizes[RX_Message[1]];
    }
    else{
      localCurrentStepSize -= stepSizes[RX_Message[1]];
    }
    // keys_1 = extractBits<20, 12>(sysState.inputs, 0, 12);
    // for (int i = 0; i < 12; i++){
    //   if (keys_1.to_ulong() != 0xFFF){
    //     if (keys_1[i] != previou_keys_1[i]){
    //       localCurrentStepSize = !keys_1[i] ? stepSizes[i] : 0;
    //     }
    //   }
    //   else{
    //     localCurrentStepSize = 0;
    //   }
    // }
    // previou_keys_1 = keys_1;
    localCurrentStepSize = localCurrentStepSize * pow(2, sysState.knobValues[2].current_knob_value-4);
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

void setup() {
  sysState.knobValues[2].current_knob_value = 4;
  //Set pin directions
  set_pin_directions();

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  //Initialise sample timer
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise CAN TX semaphore
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  //Initialise CAN Bus
  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();

  //Initialise serial port
  Serial.begin(9600);
  Serial.println("Hello World");
  
  //Create tasks
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		/* Text name for the task */
  256 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &displayUpdateHandle );	/* Pointer to store the task handle */

  TaskHandle_t decodeTaskHandle = NULL;
  xTaskCreate(
  decodeTask,		/* Function that implements the task */
  "decode",		/* Text name for the task */
  256 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdateHandle );	/* Pointer to store the task handle */

  TaskHandle_t CAN_TX_Handle = NULL;
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