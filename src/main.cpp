#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <bitset>
#include <ES_CAN.h>
#include <cmath>

#include "read_inputs.h"
#include "pin_definitions.h"
#include "read_inputs.h"
#include "wave_synth.h"

void writeToSampleBuffer(uint32_t Vout, uint32_t writeCtr){
  if (writeBuffer1){
                sampleBuffer1[writeCtr] = Vout;
              }
            else{
                sampleBuffer0[writeCtr] = Vout ;
            }
}

void backgroundCalcTask(void * pvParameters){
  static float prevfloatAmp=0;
  while(1){
    xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
    uint32_t writeCtr=0;
    while( writeCtr < SAMPLE_BUFFER_SIZE/2){
      int vol_knob_value=__atomic_load_n(&sysState.knobValues[3].current_knob_value,__ATOMIC_RELAXED);
      int version_knob_value=__atomic_load_n(&sysState.knobValues[1].current_knob_value,__ATOMIC_RELAXED);
      bool hasActiveKey=false;
      int keynum=0;
      float floatAmp=0;

      for (int i=0; i<96;i++){
        if (writeCtr< SAMPLE_BUFFER_SIZE/2){        
          bool isactive=__atomic_load_n(&notes.notes[i].active,__ATOMIC_RELAXED);
          if (isactive){
            keynum+=1;
            hasActiveKey=true;
            if (version_knob_value==8){//sawtooth
              floatAmp+=calcSawtoothAmp(&notes.notes[i].floatPhaseAcc,vol_knob_value,i);

            }
            else if(version_knob_value==7){//sin wave
              floatAmp+=getSample(notePhases[i],&notes.notes[i].floatPhaseAcc,sineTable);

            }
            else if(version_knob_value==6){//square wave
              floatAmp+=getSample(notePhases[i],&notes.notes[i].floatPhaseAcc,squareTable);
            }
            else if(version_knob_value==5){//triangle wave
              floatAmp+=getSample(notePhases[i],&notes.notes[i].floatPhaseAcc,triangleTable);
            }
            else if(version_knob_value==4){//piano
              floatAmp+=getSample(notePhases[i],&notes.notes[i].floatPhaseAcc,pianoTable);

            }
            else if(version_knob_value==3){//saxophone
              floatAmp+=getSample(notePhases[i],&notes.notes[i].floatPhaseAcc,saxophoneTable);
            }
            else if(version_knob_value==2){//flute
              floatAmp+=getSample(notePhases[i],&notes.notes[i].floatPhaseAcc,bellTable);
            }
            else if(version_knob_value==1){//alarm
              float phase=notePhases[i];
              float amp=getSample(phase,&notes.notes[i].floatPhaseAcc, squareTable);
              floatAmp+=calcHornVout(amp,vol_knob_value,i);
              
            }
            else{ //random
              floatAmp+=getSample(notePhases[i],&notes.notes[i].floatPhaseAcc,dongTable);
            }

          }
          if (i==95 && keynum>0){
              floatAmp=floatAmp/keynum;


              u_int32_t Vout=addEffects(floatAmp,prevfloatAmp,vol_knob_value,i);
              prevfloatAmp=floatAmp;
              writeToSampleBuffer(Vout, writeCtr);
              writeCtr+=1;
            }
        }
      }
      if(!hasActiveKey && writeCtr< SAMPLE_BUFFER_SIZE/2){
            writeToSampleBuffer(0,writeCtr);
            writeCtr+=1;
          }
    }
    
  }

}

void sampleISR() {
  // Serial.println("isr");
  static uint32_t readCtr = 0;
  static uint32_t metronomeCounter=0;
  int metronomespeed=__atomic_load_n(&settings.metronome.speed,__ATOMIC_RELAXED);
  bool metroOn=__atomic_load_n(&settings.metronome.on,__ATOMIC_RELAXED);
  if (sysState.posId == 0 ){
    if (readCtr == SAMPLE_BUFFER_SIZE/2) {
      readCtr = 0;
      writeBuffer1 = !writeBuffer1;
      presssedTimeCount();
      xSemaphoreGiveFromISR(sampleBufferSemaphore, NULL);
      }
    if (metronomespeed!=8 && metronomeCounter>=metronomeTime[7-metronomespeed] && metroOn){ 
      analogWrite(OUTR_PIN, 255);
      metronomeCounter=0;
    }
    else{
      if (writeBuffer1){
        
        analogWrite(OUTR_PIN, sampleBuffer0[readCtr++]);
        metronomeCounter+=1;
        }
      else{ 
        analogWrite(OUTR_PIN, sampleBuffer1[readCtr++]);
        metronomeCounter+=1;
    }
    }
  }
  else{
     if (readCtr == SAMPLE_BUFFER_SIZE/2) {
      readCtr=0;
      xSemaphoreGiveFromISR(sampleBufferSemaphore, NULL);
     }
     else{
      readCtr++;
     }
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
        Serial.println("RX: ");
        Serial.print((char)detext_RX_Message[0]);
        Serial.println(detext_RX_Message[1]);
    } while (detext_RX_Message[0] !=  'C' && detext_RX_Message[1] != 0);
    Serial.println("main board message confirmed!");
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
        delay(10);
    } while (WestDetect[0]);
    Serial.println("update west detect!");
    delay(200);
    //once west is detected, wait the message from west board
    do{
        Serial.println("waiting for west board message!");
        while (CAN_CheckRXLevel()) CAN_RX(detect_CAN_ID, detext_RX_Message);
        Serial.println(CAN_CheckRXLevel());
        Serial.print((char)detext_RX_Message[0]);
        Serial.println(detext_RX_Message[1]);
    } while (detext_RX_Message[0] != 'M');

    for (int i = 0; i < 10; i++){
        send_handshake_signal(1,1);
        delay(10);
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
    delay(500);
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
    delay(200);
    detext_TX_Message[0] = 'M';
    CAN_TX(detect_CAN_ID, detext_TX_Message);
    Serial.println("send board info to other boards!");
    Serial.print((char) detext_TX_Message[0]);
    Serial.println(detext_TX_Message[1]);
    return 0;
  }
}

void scanKeysTask(void * pvParameters) {
  Serial.println("scanKeysTask started!");
  const TickType_t xFrequency1 = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime1 = xTaskGetTickCount();
  std::bitset<12> keys;
  std::bitset<8> current_knobs;
  std::bitset<12> previou_keys("111111111111");
  std::bitset<8> previous_knobs("00000000");
  std::bitset<1> old_WestDetect;
  std::bitset<1> old_EastDetect;
  while (1){ 
    vTaskDelayUntil( &xLastWakeTime1, xFrequency1);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    xSemaphoreTake(notes.mutex, portMAX_DELAY);
    sysState.inputs = readInputs();

    keys = extractBits<28, 12>(sysState.inputs, 0, 12);
    current_knobs = extractBits<28, 8>(sysState.inputs, 12, 8);
    sysState.WestDetect = extractBits<28, 1>(sysState.inputs, 23, 1);
    sysState.EastDetect = extractBits<28, 1>(sysState.inputs, 27, 1);
    updateKnob(sysState.knobValues, previous_knobs, current_knobs);

    //if there is nothing on the west and east before, but now there is something on the west or east
    //then update the posId
    if (old_EastDetect[0] && old_WestDetect[0] && (!sysState.WestDetect[0])){
      Serial.println("request posId");
      delay(100);
      TX_Message[0] = 'N';
      TX_Message[1] = sysState.local_boardId;
      xQueueSend( msgOutQ, const_cast<uint8_t*>(TX_Message), portMAX_DELAY);
    }

    // if nothing on west and east then local play mode
    if (sysState.EastDetect[0] && sysState.WestDetect[0]){
      sysState.posId = 0;
      sysState.knobValues[2].current_knob_value = sysState.posId + 3;
      int tune=sysState.knobValues[2].current_knob_value;
      for (int i = 0; i < 12; i++){
        if (keys.to_ulong() != 0xFFF){
          if (!keys[i]) {
            notes.notes[(tune-1)*12+i].active = true;
            // Serial.print((tune-1)*12+i);
            // Serial.print(i);
            // Serial.print("pressed");
          // localCurrentStepSize1 = stepSizes[i];

          }
          else{
            notes.notes[(tune-1)*12+i].active =false;
          }
        }
        else{
          notes.notes[(tune-1)*12+i].active =false;
          //localCurrentStepSize1 = 0;
          // Serial.print("released");

        }
      }
      // localCurrentStepSize1 = localCurrentStepSize1 * pow(2, sysState.knobValues[2].current_knob_value - 4);
      // Serial.println(localCurrentStepSize1);
      // __atomic_store_n(&currentStepSize, localCurrentStepSize1, __ATOMIC_RELAXED);
    }
    // transmit mode, do not play note locally
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
    old_WestDetect = sysState.WestDetect;
    old_EastDetect = sysState.EastDetect;
    xSemaphoreGive(sysState.mutex);
    xSemaphoreGive(notes.mutex);
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

    u8g2.setCursor(100,30);
    u8g2.print(sysState.WestDetect[0]);
    u8g2.print(sysState.EastDetect[0]);

    u8g2.sendBuffer();
	  
    digitalToggle(LED_BUILTIN);
  }
}

void decodeTask(void * pvParameters) {
  Serial.println("decodeTask started!");
  while (1) {
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    int tune_knob_value=__atomic_load_n(&sysState.knobValues[2].current_knob_value,__ATOMIC_RELAXED);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    xSemaphoreTake(notes.mutex, portMAX_DELAY);

    if (sysState.posId == 0){
      if (RX_Message[0] == 'P'){
        sysState.inputs[RX_Message[1]] = 0;
        notes.notes[(RX_Message[2]-1)*12+RX_Message[1]].active = true;

      }
      else if (RX_Message[0] == 'R'){
        sysState.inputs[RX_Message[1]] = 1;
        notes.notes[(RX_Message[2]-1)*12+RX_Message[1]].active = false;
      }
    }
    // // if board 1 receives a message from board 0, it will send the message back to the board 0
    else if(RX_Message[3] == 0 && sysState.posId == 1
          && (RX_Message[0] == 'R' || RX_Message[0] == 'P')){
        RX_Message[3] = 1;
        xQueueSend( msgOutQ, const_cast<uint8_t*>(RX_Message), portMAX_DELAY);
    }
    
    if (RX_Message[0] == 'N'){
      Serial.println("new board request receriaved!");
      TX_Message[0] = 'U';
      TX_Message[1] = sysState.posId;
      TX_Message[2] = RX_Message[1];
      xQueueSend( msgOutQ, const_cast<uint8_t*>(TX_Message), portMAX_DELAY);
    }

    if (RX_Message[0] == 'U' && RX_Message[2] == sysState.local_boardId){
      if (RX_Message[1] >= sysState.posId){
        sysState.posId = RX_Message[1] + 1;
        sysState.knobValues[2].current_knob_value = sysState.posId + 3;
        Serial.print("update posId: ");
        Serial.println(sysState.posId);
      }
    }

    Serial.print("RX: ");
    Serial.print((char) RX_Message[0]);
    Serial.print(RX_Message[1]);
    Serial.print(RX_Message[2]);
    Serial.println();
    xSemaphoreGive(notes.mutex);
    xSemaphoreGive(sysState.mutex);

  }
}

void CAN_TX_Task (void * pvParameters) {
  Serial.println("CAN_TX_Task started!");
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

void setup() {
  sysState.knobValues[2].current_knob_value = 4;
  sysState.knobValues[3].current_knob_value = 6;
  //Set pin directions
  generatePhaseLUT();
  set_pin_directions();
  set_notes();
  init_settings();

  initial_display();

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  //Initialise sample timer
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise CAN TX semaphore
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);
  sampleBufferSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(sampleBufferSemaphore);

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
  delay(200);
  initial_display();
  Serial.print("posId: ");
  Serial.println(sysState.posId);
  sysState.knobValues[2].current_knob_value = sysState.posId + 3;
  Serial.print("UID: ");
  Serial.println(sysState.local_boardId);

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
  &decodeTaskHandle );	/* Pointer to store the task handle */

  xTaskCreate(
  CAN_TX_Task,		/* Function that implements the task */
  "CAN_TX",		/* Text name for the task */
  256 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &CAN_TX_Handle );	/* Pointer to store the task handle */


  TaskHandle_t BackCalc_Handle = NULL;
  xTaskCreate(
  backgroundCalcTask,		/* Function that implements the task */
  "BackCalc",		/* Text name for the task */
  256 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  5,			/* Task priority */
  &BackCalc_Handle );	/* Pointer to store the task handle */
  
  notes.mutex = xSemaphoreCreateMutex();
  sysState.mutex = xSemaphoreCreateMutex(); //Create mutex
  vTaskStartScheduler();
}

void loop() {
}