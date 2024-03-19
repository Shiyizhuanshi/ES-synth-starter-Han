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
  uint8_t local_boardId = HAL_GetUIDw0();
  int posId = 0;
  std::bitset<1> WestDetect;
  std::bitset<1> EastDetect;
  bool singleMode = true;
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
const int SAMPLE_BUFFER_SIZE =4400;
uint8_t sampleBuffer0[SAMPLE_BUFFER_SIZE/2];
uint8_t sampleBuffer1[SAMPLE_BUFFER_SIZE/2];
SemaphoreHandle_t sampleBufferSemaphore;
volatile bool writeBuffer1 = false;

void writeToSampleBuffer(uint32_t Vout, uint32_t writeCtr){
  if (writeBuffer1){
                sampleBuffer1[writeCtr] = Vout;
              }
            else{
                sampleBuffer0[writeCtr] = Vout ;
            }
}

void backgroundCalcTask(void * pvParameters){
  static uint32_t  phaseAcc=0;
  // static float sinAcc=0;
  // static float saxAcc=0;
  static float prevfloatAmp=0;
  while(1){
    Serial.print("inbackcalc");
	  xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
    uint32_t writeCtr=0;
   
    
  while( writeCtr < SAMPLE_BUFFER_SIZE/2){

      
      int vol_knob_value=__atomic_load_n(&sysState.knobValues[3].current_knob_value,__ATOMIC_RELAXED);
      // int tune_knob_value=__atomic_load_n(&sysState.knobValues[2].current_knob_value,__ATOMIC_RELAXED);
      int version_knob_value=__atomic_load_n(&sysState.knobValues[1].current_knob_value,__ATOMIC_RELAXED);
      bool hasActiveKey=false;
      int keynum=0;
      // float sinAmp=0;
      // float triAmp=0;
      // float pianoAmp=0;
      float floatAmp=0;
      for (int i = 0; i < 96; i++) {
        // Serial.print(i);
        if (writeCtr< SAMPLE_BUFFER_SIZE/2){

          
          bool isactive=__atomic_load_n(&notes.notes[i].active,__ATOMIC_RELAXED);

          if (version_knob_value ==8){
            
            if (isactive) {
                // uint32_t phaseAcc=__atomic_load_n(&notes.notes[i].phaseAcc,__ATOMIC_RELAXED);
                uint32_t stepSize=__atomic_load_n(&stepSizes[i%12],__ATOMIC_RELAXED);
                
                hasActiveKey=true;
                int tune=int(ceil(1/12));
                if ((tune-4)>=0){
                  phaseAcc+=stepSize << (tune-4);}
                else{
                  phaseAcc+=stepSize >> -(tune-4);
                }
                uint32_t Vout = (phaseAcc >> 24) - 128;
                Vout = (Vout >> (8 - vol_knob_value))+128 ;
                // uint32_t Vout =calcSawtoothVout(phaseAcc,vol_knob_value,i);
                
                writeToSampleBuffer(Vout,writeCtr);
                writeCtr+=1;
                
                // notes.notes[i].phaseAcc=phaseAcc;
            }
          }
          else if(version_knob_value ==7){

              if (isactive) {
                
                keynum+=1;
                hasActiveKey=true;
                floatAmp+=getSample(notePhases[i],&notes.notes[i].floatPhaseAcc,sineTable);

              }
              if (i==11 && keynum>0){
                floatAmp=floatAmp/keynum;
                floatAmp=calcNoEnvelopeVout(floatAmp,vol_knob_value);
                writeToSampleBuffer(int(floatAmp)  , writeCtr);
                writeCtr+=1;
            }

          }
          else if (version_knob_value ==6){


            // }
            if (isactive) {
                // testsinAcc=notes.notes[i].sinAcc;
                keynum+=1;
                hasActiveKey=true;

                floatAmp+=calcOtherVout( getSample(notePhases[i],&notes.notes[i].floatPhaseAcc,sineTable),vol_knob_value,i);

                
              }
            if (i==11 && keynum>0){
              floatAmp=floatAmp/keynum;
              // floatAmp=calcNoEnvelopeVout(floatAmp,vol_knob_value);
              writeToSampleBuffer(int(floatAmp)  , writeCtr);
              writeCtr+=1;
            }

          }
          else if (version_knob_value ==5){
            // float testsinAcc=0;
            if (isactive) {
                // testsinAcc=notes.notes[i].sinAcc;
                keynum+=1;
                hasActiveKey=true;
                // floatAmp+=generateSin(sinPhases[i],&notes.notes[i].sinAcc);
                floatAmp+=getSample(notePhases[i],&notes.notes[i].floatPhaseAcc,pianoTable);
                // testsinAcc+=sinPhases[i];
                // if (testsinAcc>=M_PI){
                //   testsinAcc-=M_PI;
                // }
                // floatAmp+=sin(testsinAcc);
                // notes.notes[i].sinAcc=testsinAcc;
                
              }
            if (i==11 && keynum>0){
              floatAmp=floatAmp/keynum;
              // floatAmp= lowPassFilter(floatAmp,prevfloatAmp,500.0);
              // prevfloatAmp=floatAmp;
              uint32_t Vout = static_cast<uint32_t>(floatAmp*255)-128;
                Vout = Vout >> (8 - vol_knob_value);
              writeToSampleBuffer(Vout+128, writeCtr);
              writeCtr+=1;
            }
          

          }
          else if (version_knob_value ==4){
             if (isactive){
            keynum+=1;
            hasActiveKey=true;
            float phase=notePhases[i];
            float amp=getSample(phase,&notes.notes[i].floatPhaseAcc, squareTable);
            floatAmp+=calcHornVout(amp,vol_knob_value,i);

             }
          
          if (i==11 && keynum>0){
              floatAmp=floatAmp/keynum;
              uint32_t Vout = static_cast<uint32_t>(floatAmp*127)-128;
                Vout = Vout >> (8 - vol_knob_value);
                writeToSampleBuffer(Vout+128, writeCtr);
                writeCtr+=1;
            }

          }
          else if (version_knob_value ==3){
             if (isactive){
            keynum+=1;
            hasActiveKey=true;
            float phase=notePhases[i];
            float amp=getSample(phase,&notes.notes[i].floatPhaseAcc, saxophoneTable);
            floatAmp+=calcPianoVout(amp,vol_knob_value,i);
             }
          
          if (i==11 && keynum>0){
              floatAmp=floatAmp/keynum;
              uint32_t Vout = static_cast<uint32_t>(floatAmp*127)-128;
                Vout = Vout >> (8 - vol_knob_value);
                writeToSampleBuffer(Vout+128, writeCtr);
                writeCtr+=1;
            }

          }
          else if (version_knob_value ==2){
             if (isactive){
            keynum+=1;
            hasActiveKey=true;
            float phase=notePhases[i];
            floatAmp+=getSample(phase,&notes.notes[i].floatPhaseAcc, saxophoneTable);
             }
          
          if (i==11 && keynum>0){
              floatAmp=floatAmp/keynum;
              floatAmp+=generateLFO(1);
              uint32_t Vout = static_cast<uint32_t>(floatAmp*127)-128;
                Vout = Vout >> (8 - vol_knob_value);
                writeToSampleBuffer(Vout+128, writeCtr);
                writeCtr+=1;
            }

          }

          else{
            if (isactive){
              keynum+=1;
              hasActiveKey=true;
              floatAmp+=getSample(notePhases[i],&notes.notes[i].floatPhaseAcc,triangleTable);

            }
            if (i==11 && keynum>0){
              floatAmp=floatAmp/keynum;
              uint32_t Vout = static_cast<uint32_t>(floatAmp*255)-128;
                Vout = Vout >> (8 - vol_knob_value);
                writeToSampleBuffer(Vout+128, writeCtr);
                writeCtr+=1;
            }

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
  static uint32_t readCtr = 0;
  if (sysState.posId == 0 ){
    
    int metronome_knob_value=__atomic_load_n(&sysState.knobValues[0].current_knob_value,__ATOMIC_RELAXED);
    static uint32_t metronomeCounter=0;
    if (readCtr == SAMPLE_BUFFER_SIZE/2) {
      readCtr = 0;
      writeBuffer1 = !writeBuffer1;
      presssedTimeCount();
      xSemaphoreGiveFromISR(sampleBufferSemaphore, NULL);
      }
    if (metronome_knob_value!=8 && metronomeCounter>=metronomeTime[7-metronome_knob_value]){ 
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
  volatile uint32_t localCurrentStepSize1;
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
      for (int i = 0; i < 12; i++){
        if (keys.to_ulong() != 0xFFF){
          if (!keys[i]) {
            notes.notes[(sysState.knobValues[2].current_knob_value-1)*12+i].active = true;
          // localCurrentStepSize1 = stepSizes[i];

          }
        }
        else{
          notes.notes[(sysState.knobValues[2].current_knob_value-1)*12+i].active = true;
          //localCurrentStepSize1 = 0;

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
  std::bitset<12> keys_1;
  std::bitset<12> previou_keys_1("111111111111");
  volatile uint32_t localCurrentStepSize2;
  while (1) {
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    int tune_knob_value=__atomic_load_n(&sysState.knobValues[2].current_knob_value,__ATOMIC_RELAXED);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    xSemaphoreTake(notes.mutex, portMAX_DELAY);

    if (sysState.posId == 0){
      if (RX_Message[0] == 'P'){
        sysState.inputs[RX_Message[1]] = 0;
        notes.notes[(RX_Message[2]-1)*4+RX_Message[1]].active = true;

      }
      else if (RX_Message[0] == 'R'){
        sysState.inputs[RX_Message[1]] = 1;
        notes.notes[(RX_Message[2]-1)*4+RX_Message[1]].active = false;
      }

      // keys_1 = extractBits<28, 12>(sysState.inputs, 0, 12);
      // for (int i = 0; i < 12; i++){
      //   if (keys_1.to_ulong() != 0xFFF){
      //     if (keys_1[i] != previou_keys_1[i]){
      //       localCurrentStepSize2 = !keys_1[i] ? stepSizes[i] : 0;
      //     }
      //   }
      //   else{
      //     localCurrentStepSize2 = 0;
      //   }
      // }
      // previou_keys_1 = keys_1;
      // localCurrentStepSize2 = localCurrentStepSize2 * pow(2, RX_Message[2] - 4);

      // Serial.println(localCurrentStepSize2);
      // __atomic_store_n(&currentStepSize, localCurrentStepSize2, __ATOMIC_RELAXED);
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
  &displayUpdateHandle );	/* Pointer to store the task handle */

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
  
  sysState.mutex = xSemaphoreCreateMutex();
  sysState.mutex = xSemaphoreCreateMutex(); //Create mutex
  vTaskStartScheduler();
}

void loop() {
}