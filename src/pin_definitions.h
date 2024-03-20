// pin_definitions.h

#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <string>

#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

const uint32_t stepSizes[12] = {
  51149156,  //C
  54190643,  //C#
  57412986,  //D
  60826940,  //D#
  64443898,  //E
  68275931,  //F
  72335830,  //F#
  76637142,  //G
  81194224,  //G#
  86022284,  //A
  91137435,  //A#
  96556749   //B
};

void set_pin_directions(){
    pinMode(RA0_PIN, OUTPUT);
    pinMode(RA1_PIN, OUTPUT);
    pinMode(RA2_PIN, OUTPUT);
    pinMode(REN_PIN, OUTPUT);
    pinMode(OUT_PIN, OUTPUT);
    pinMode(OUTL_PIN, OUTPUT);
    pinMode(OUTR_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(C0_PIN, INPUT);
    pinMode(C1_PIN, INPUT);
    pinMode(C2_PIN, INPUT);
    pinMode(C3_PIN, INPUT);
    pinMode(JOYX_PIN, INPUT);
    pinMode(JOYY_PIN, INPUT);
}
std::array<std::string, 12> noteNames = {
  "C",
  "C#",
  "D",
  "D#",
  "E",
  "F",
  "F#",
  "G",
  "G#",
  "A",
  "A#",
  "B"
};

const std::size_t inputSize = 28;

struct knob{
  int current_knob_value = 8;
  int lastIncrement = 0;
  bool clickState = 0;
};
struct ADSR{
  bool on;
  int attack;
  int decay;
  int sustain;
  //int release;
};
struct LFO{
  bool on;
  int freq;
  int reduceLFOVolume;
};
struct Metronome{
  bool on;
  int speed;
};
struct Lowpass{
  int on;
  int freq;
};
struct Fade{
  int on;
  int sustainTime;
  int fadeSpeed;
};
struct {
  Fade fade;
  Lowpass lowpass;
  LFO lfo;
  ADSR adsr;
  Metronome metronome;
}settings;



void init_settings(){
  settings.fade.on=false;
  settings.fade.fadeSpeed=2;
  settings.fade.sustainTime=3;
  settings.adsr.on=false;
  settings.adsr.attack=1;
  settings.adsr.decay=4;
  settings.adsr.sustain=8;
  settings.lowpass.on=false;
  settings.lowpass.freq=500;
  settings.metronome.on=false;
  settings.metronome.speed=8;
  settings.lfo.freq=20;
  settings.lfo.on=false;
  settings.lfo.reduceLFOVolume=2;
}

//Constants
const uint32_t interval = 100; //Display update interval

uint32_t ID = 0x123; //CAN ID
uint8_t RX_Message[8] = {0};  //CAN RX message
volatile uint8_t TX_Message[8] = {0}; //CAN TX message
std::string movement;
//Create message input and output queues
//36 messages of 8 bytes, each message takes around 0.7ms to process
QueueHandle_t msgInQ = xQueueCreate(36,8);; // Message input queue
QueueHandle_t msgOutQ = xQueueCreate(36,8);; // Message output queue

SemaphoreHandle_t CAN_TX_Semaphore; //CAN TX semaphore

std::string bottomBar_menu[4] = {"Menu", "Test", "Tone", "Vol"};
std::string waveNames[9] = {"Saw", "Sin", "Squ", "Tri", "Pia", "Saxo", "Bell", "Alar", "None"};
std::string menu_first_level[6] = {"Met", "Fade", "LFO", "ADSR", "LPF", "exit"};

//Struct to hold system state
struct State{
  std::bitset<inputSize> inputs;
  SemaphoreHandle_t mutex;  
  std::array<knob, 4> knobValues;
  int joystickState = 0;
} sysState;

volatile uint32_t currentStepSize;

const uint32_t sampleRate = 22000;  //Sample rate

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

#endif  // PIN_DEFINITIONS_H
