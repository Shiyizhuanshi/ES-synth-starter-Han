#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include <cmath>

#include "read_inputs.h"
#include "pin_definitions.h"
#include "menu.h"

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

void scanJoystickTask(void * pvParameters) {
  const TickType_t xFrequency3 = 99/portTICK_PERIOD_MS;
  int joystickX;
  int joystickY;
  int origin = 490;
  int deadZone = 150;
  
  std::string previous_movement = "origin";
  std::string current_movement;
  TickType_t xLastWakeTime3 = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil( &xLastWakeTime3, xFrequency3);
    joystickX = analogRead(JOYX_PIN);
    joystickY = analogRead(JOYY_PIN);
    // Serial.print(joystickX);
    // Serial.print(", ");
    // Serial.println(joystickY);
    if (abs(joystickX - origin) <= deadZone && abs(joystickY - origin) <= deadZone){
      // Serial.println("In deadzone");
      current_movement = "origin";
    }
    else if (joystickX - origin > deadZone){
      // Serial.println("Left");
      current_movement = "left";
    }
    else if (joystickX - origin < -deadZone){
      // Serial.println("Right");
      current_movement = "right";
    }
    else if (joystickY - origin > deadZone){
      // Serial.println("Down");
      current_movement = "down";
    }
    else if (joystickY - origin < -deadZone){
      // Serial.println("Up");
      current_movement = "up";
    }

    if (current_movement != previous_movement && current_movement == "origin"){
      movement = previous_movement;
    }
    else{movement = "origin";}
    // Serial.println(movement.c_str());
    previous_movement = current_movement;
  }
}

void scanKeysTask(void * pvParameters) {
  // volatile uint32_t localCurrentStepSize;
  const TickType_t xFrequency1 = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime1 = xTaskGetTickCount();
  std::bitset<12> keys;
  std::bitset<8> current_knobs;
  std::bitset<4> current_knobs_click;

  std::bitset<1> currentJoystickState{"0"};
  std::bitset<1> oldJoystickState{"0"};

  std::bitset<12> previous_keys("111111111111");
  std::bitset<8> previous_knobs("00000000");
  std::bitset<4> previous_knobs_click("0000");
  while (1){ 
    vTaskDelayUntil( &xLastWakeTime1, xFrequency1);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = readInputs();

    keys = extractBits<inputSize, 12>(sysState.inputs, 0, 12);
    current_knobs = extractBits<inputSize, 8>(sysState.inputs, 12, 8);
    current_knobs_click = extractBits<inputSize, 4>(sysState.inputs, 20, 2).to_ulong() << 2 | extractBits<inputSize, 4>(sysState.inputs, 24, 2).to_ulong();
    currentJoystickState = extractBits<inputSize, 1>(sysState.inputs, 22, 1).to_ulong();
    if (currentJoystickState != oldJoystickState && currentJoystickState == 0){
      sysState.joystickState = !sysState.joystickState;
    }
    updateKnob(sysState.knobValues, previous_knobs, current_knobs, previous_knobs_click, current_knobs_click);

    xSemaphoreGive(sysState.mutex);

    for (int i = 0; i < 12; i++){
      // Decode keys
      if (keys[i] != previous_keys[i]){
        TX_Message[0] = keys[i] ? 'R' : 'P';
        TX_Message[1] = i;
        TX_Message[2] = 4;
        // CAN_TX(0x123, const_cast<uint8_t*>(TX_Message));
        xQueueSend( msgOutQ, const_cast<uint8_t*>(TX_Message), portMAX_DELAY);
      }
    }

    previous_keys = keys;
    previous_knobs = current_knobs;
    previous_knobs_click = current_knobs_click;
    oldJoystickState = currentJoystickState;
    // __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency2 = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime2 = xTaskGetTickCount();
  static uint32_t count = 0;
  int posX = 0;
  int posY = 0;
  int index = 0;
  int iconsPos[3] = {624, 416, 87};
  int stay_time = 1000;
  int counter = 0;
  std::bitset<2> previous_knob1("00");
  std::bitset<2> previous_knob2("00");
  std::bitset<2> previous_knob3("00");
  while (1) {
    vTaskDelayUntil( &xLastWakeTime2, xFrequency2);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    if (counter != 0){
      counter --;
    }
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    
    if(sysState.knobValues[0].clickState){
      u8g2.setFont(u8g2_font_6x10_tr);
      if (movement == "down"){
        index += 3;
      }
      else if (movement == "up"){
        index -= 3;
      }
      else if (movement == "right"){
        index += 1;
      }
      else if (movement == "left"){
        index -= 1;
      }
      index = constrain(index, 0, 5);

      for (int i = 0; i < 6; i++){
        if (i< 3){
          u8g2.drawStr(10 + 35 * i, 14, menu_first_level[i].c_str());
        }
        else{
          u8g2.drawStr(10 + 35 * (i-3), 25, menu_first_level[i].c_str());
        }
        if (index == i){
          int x_pos = (i < 3) ? 10 + 35 * i : 10 + 35 * (i-3);
          int y_pos = (i < 3) ? 5 : 15;
          u8g2.drawFrame(x_pos-3, y_pos, 32, 12);
        }
      }

      if (sysState.joystickState){
        menu(index);
      }
      // for (int i = 0; i < 3; i++){
      //   u8g2.drawGlyph(40*i+10, 28, iconsPos[i]);
      //   if (index == i ){
      //     u8g2.drawFrame(40*i+10-3, 5, 27, 27);
      //   }
      // }
      // if (sysState.joystickState){
      //   u8g2.clearBuffer();
      //   u8g2.setFont(u8g2_font_ncenB08_tr);
      //   u8g2.setCursor(10, 10);
      //   u8g2.print("Joystick pressed");
      // }
    }
    else{
      u8g2.setCursor(45, 10);
      u8g2.print("StackY");
      u8g2.setFont(u8g2_font_5x8_tr);
      for (int i = 0; i < 4; i++){
        u8g2.drawFrame(8+30*i, 20, 25, 20);
        if (i !=0 && 
           (extractBits<inputSize, 2>(sysState.inputs, 12, 2) != previous_knob3 ||
            extractBits<inputSize, 2>(sysState.inputs, 14, 2) != previous_knob2 ||
            extractBits<inputSize, 2>(sysState.inputs, 16, 2) != previous_knob1)){
          counter = stay_time/100;
        }

        if (counter != 0  && i ==3){
          u8g2.drawStr(10+30*i, 29, std::to_string(sysState.knobValues[3].current_knob_value).c_str());
        }
        else if (counter != 0 && i ==2 ){
          u8g2.drawStr(10+30*i, 29, std::to_string(sysState.knobValues[2].current_knob_value).c_str());
        }
        else if (counter != 0 && i ==1 ){
          u8g2.drawStr(10+30*i, 29, waveNames[sysState.knobValues[1].current_knob_value].c_str());
        }
        else{
          u8g2.drawStr(10+30*i, 29, bottomBar_menu[i].c_str());
        }
      }
      //display pressed keys
      std::vector<std::string> pressedKeys;
      for (int i = 0; i < 12; i++){
        if (sysState.inputs[i] == 0){
          pressedKeys.push_back(noteNames[i]);
        }
      }
      for (int i = 0; i < pressedKeys.size(); i++){
        u8g2.setCursor(10+ 10*i, 18);
        u8g2.print(pressedKeys[i].c_str());
      }
    }
    previous_knob1 = extractBits<inputSize, 2>(sysState.inputs, 16, 2);
    previous_knob2 = extractBits<inputSize, 2>(sysState.inputs, 14, 2);
    previous_knob3 = extractBits<inputSize, 2>(sysState.inputs, 12, 2);
    xSemaphoreGive(sysState.mutex);
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
  sysState.knobValues[3].current_knob_value = 0;

  //Set pin directions
  set_pin_directions();
  init_settings();
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

  TaskHandle_t scanJoystick_Handle = NULL;
  xTaskCreate(
  scanJoystickTask,		/* Function that implements the task */
  "scanJoystick",		/* Text name for the task */
  256 ,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &scanJoystick_Handle );	/* Pointer to store the task handle */
  
  sysState.mutex = xSemaphoreCreateMutex(); //Create mutex
  vTaskStartScheduler();

}

void loop() {
}