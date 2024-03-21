#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <string>

#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H
#define SAMPLE_RATE 22000

//constants
const uint32_t sampleRate = 22000;  //Sample rate

uint32_t ID = 0x123; //CAN ID
uint8_t RX_Message[8] = {0};  //CAN RX message
volatile uint8_t TX_Message[8] = {0}; //CAN TX message

//Create message input and output queues
//36 messages of 8 bytes, each message takes around 0.7ms to process
QueueHandle_t msgInQ = xQueueCreate(60,8);; // Message input queue
QueueHandle_t msgOutQ = xQueueCreate(60,8);; // Message output queue

SemaphoreHandle_t CAN_TX_Semaphore; //CAN TX semaphore

//Create task handles
TaskHandle_t scanKeysHandle = NULL;
TaskHandle_t displayUpdateHandle = NULL;
TaskHandle_t decodeTaskHandle = NULL;
TaskHandle_t CAN_TX_Handle = NULL;
TaskHandle_t BackCalc_Handle = NULL;
TaskHandle_t scanJoystick_Handle = NULL;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);


const int SAMPLE_BUFFER_SIZE =2200;
uint8_t sampleBuffer0[SAMPLE_BUFFER_SIZE/2];
uint8_t sampleBuffer1[SAMPLE_BUFFER_SIZE/2];
SemaphoreHandle_t sampleBufferSemaphore;
volatile bool writeBuffer1 = false;

const std::size_t inputSize = 28;
std::string movement;

std::string bottomBar_menu[4] = {"Menu", "Wave", "Tone", "Vol"};
std::string waveNames[9] = {"Saw", "Sin", "Squ", "Tri", "Pia", "Saxo", "Bell", "Alar", "None"};
std::string menu_first_level[6] = {"Met", "Fade", "LFO", "ADSR", "LPF", "exit"};

struct knob{
  int current_knob_value = 8;
  int lastIncrement = 0;
  bool clickState = 0;
};

//Struct to hold system state
struct {
  std::bitset<inputSize> inputs;
  SemaphoreHandle_t mutex;  
  std::array<knob, 4> knobValues;
  uint8_t local_boardId = HAL_GetUIDw0();
  std::bitset<1> WestDetect;
  std::bitset<1> EastDetect;
  std::string currentMenu = "Main";
  int joystickState = 0;
  bool singleMode = true;
  int posId = 0;
} sysState;

struct note {
  uint32_t stepSize;
  uint32_t phaseAcc;
  float floatPhaseAcc;
  int pressedCount;
  bool active;

};

struct {
  std::array<note, 96> notes;
  SemaphoreHandle_t mutex;  
} notes;

struct ADSR{
  bool on;
  int attack;
  int decay;
  int sustain;
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
struct setting{
  Metronome metronome;
  Fade fade;
  LFO lfo;
  ADSR adsr;
  Lowpass lowpass;
  int volume;
  int tune;
  int waveIndex;
  SemaphoreHandle_t mutex;  
}settings;

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

const uint32_t metronomeTime[7] ={
  22000,
  11000,
  5500,
  2750,
  1325,
  610,
  300
};

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
float noteFrequencies[96]={
    32.70,   // C1
    34.65,   // C#1/Db1
    36.71,   // D1
    38.89,   // D#1/Eb1
    41.20,   // E1
    43.65,   // F1
    46.25,   // F#1/Gb1
    49.00,   // G1
    51.91,   // G#1/Ab1
    55.00,   // A1
    58.27,   // A#1/Bb1
    61.74,   // B1
    65.41,   // C2
    69.30,   // C#2/Db2
    73.42,   // D2
    77.78,   // D#2/Eb2
    82.41,   // E2
    87.31,   // F2
    92.50,   // F#2/Gb2
    98.00,   // G2
    103.83,  // G#2/Ab2
    110.00,  // A2
    116.54,  // A#2/Bb2
    123.47,  // B2
    130.81,  // C3
    138.59,  // C#3/Db3
    146.83,  // D3
    155.56,  // D#3/Eb3
    164.81,  // E3
    174.61,  // F3
    185.00,  // F#3/Gb3
    196.00,  // G3
    207.65,  // G#3/Ab3
    220.00,  // A3
    233.08,  // A#3/Bb3
    246.94,  // B3
    261.63,  // C4
    277.18,  // C#4/Db4
    293.66,  // D4
    311.13,  // D#4/Eb4
    329.63,  // E4
    349.23,  // F4
    369.99,  // F#4/Gb4
    392.00,  // G4
    415.30,  // G#4/Ab4
    440.00,  // A4
    466.16,  // A#4/Bb4
    493.88,  // B4
    523.25,  // C5
    554.37,  // C#5/Db5
    587.33,  // D5
    622.25,  // D#5/Eb5
    659.25,  // E5
    698.46,  // F5
    739.99,  // F#5/Gb5
    783.99,  // G5
    830.61,  // G#5/Ab5
    880.00,  // A5
    932.33,  // A#5/Bb5
    987.77,  // B5
    1046.50, // C6
    1108.73, // C#6/Db6
    1174.66, // D6
    1244.51, // D#6/Eb6
    1318.51, // E6
    1396.91, // F6
    1479.98, // F#6/Gb6
    1567.98, // G6
    1661.22, // G#6/Ab6
    1760.00, // A6
    1864.66, // A#6/Bb6
    1975.53,  // B6
    2093.00,  // C7
    2217.46,  // C#7
    2349.32,  // D7
    2489.02,  // D#7
    2637.02,  // E7
    2793.83,  // F7
    2959.96,  // F#7
    3135.96,  // G7
    3322.44,  // G#7
    3520.00,  // A7
    3729.31,  // A#7
    3951.07,   // B7
    4186.01,  // C8
    4434.92,  // C#8
    4698.63,  // D8
    4978.03,  // D#8
    5274.04,  // E8
    5587.65,  // F8
    5919.91,  // F#8
    6271.93,  // G8
    6644.88,  // G#8
    7040.00,  // A8
    7458.62,  // A#8
    7902.13   // B8
};

float notePhases[96];

void generatePhaseLUT(){
  for (int i=0;i<96;i++){
    notePhases[i]=noteFrequencies[i]*M_PI/SAMPLE_RATE;
  }
}

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
  settings.volume=4;
  settings.tune=4;
  settings.waveIndex=0;
}


void set_notes(){
  for (int i = 0; i < 96; i++){
    // notes.notes[i].stepSize = stepSizes[i];
    notes.notes[i].phaseAcc = 0;
    notes.notes[i].floatPhaseAcc=0;
    notes.notes[i].active = false;
    // Serial.println(notes.notes[i].stepSize);
  }
}

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

float pianoTable[256] = {
    0.000000, 0.024541, 0.049068, 0.073565, 0.098017, 0.122411, 0.146730, 0.170962,
    0.195090, 0.219101, 0.242980, 0.266713, 0.290285, 0.313682, 0.336890, 0.359895,
    0.382683, 0.405241, 0.427555, 0.449611, 0.471397, 0.492898, 0.514103, 0.534998,
    0.555570, 0.575808, 0.595699, 0.615232, 0.634393, 0.653173, 0.671559, 0.689541,
    0.707107, 0.724247, 0.740951, 0.757209, 0.773010, 0.788346, 0.803208, 0.817585,
    0.831470, 0.844854, 0.857729, 0.870087, 0.881921, 0.893224, 0.903989, 0.914210,
    0.923880, 0.932993, 0.941544, 0.949528, 0.956940, 0.963776, 0.970031, 0.975702,
    0.980785, 0.985278, 0.989177, 0.992480, 0.995185, 0.997290, 0.998795, 0.999699,
    1.000000, 0.999699, 0.998795, 0.997290, 0.995185, 0.992480, 0.989177, 0.985278,
    0.980785, 0.975702, 0.970031, 0.963776, 0.956940, 0.949528, 0.941544, 0.932993,
    0.923880, 0.914210, 0.903989, 0.893224, 0.881921, 0.870087, 0.857729, 0.844854,
    0.831470, 0.817585, 0.803208, 0.788346, 0.773010, 0.757209, 0.740951, 0.724247,
    0.707107, 0.689541, 0.671559, 0.653173, 0.634393, 0.615232, 0.595699, 0.575808,
    0.555570, 0.534998, 0.514103, 0.492898, 0.471397, 0.449611, 0.427555, 0.405241,
    0.382683, 0.359895, 0.336890, 0.313682, 0.290285, 0.266713, 0.242980, 0.219101,
    0.195090, 0.170962, 0.146730, 0.122411, 0.098017, 0.073565, 0.049068, 0.024541,
    0.000000, -0.024541, -0.049068, -0.073565, -0.098017, -0.122411, -0.146730, -0.170962,
    -0.195090, -0.219101, -0.242980, -0.266713, -0.290285, -0.313682, -0.336890, -0.359895,
    -0.382683, -0.405241, -0.427555, -0.449611, -0.471397, -0.492898, -0.514103, -0.534998,
    -0.555570, -0.575808, -0.595699, -0.615232, -0.634393, -0.653173, -0.671559, -0.689541,
    -0.707107, -0.724247, -0.740951, -0.757209, -0.773010, -0.788346, -0.803208, -0.817585,
    -0.831470, -0.844854, -0.857729, -0.870087, -0.881921, -0.893224, -0.903989, -0.914210,
    -0.923880, -0.932993, -0.941544, -0.949528, -0.956940, -0.963776, -0.970031, -0.975702,
    -0.980785, -0.985278, -0.989177, -0.992480, -0.995185, -0.997290, -0.998795, -0.999699,
    -1.000000, -0.999699, -0.998795, -0.997290, -0.995185, -0.992480, -0.989177, -0.985278,
    -0.980785, -0.975702, -0.970031, -0.963776, -0.956940, -0.949528, -0.941544, -0.932993,
    -0.923880, -0.914210, -0.903989, -0.893224, -0.881921, -0.870087, -0.857729, -0.844854,
    -0.831470, -0.817585, -0.803208, -0.788346, -0.773010, -0.757209, -0.740951, -0.724247,
    -0.707107, -0.689541, -0.671559, -0.653173, -0.634393, -0.615232, -0.595699, -0.575808,
    -0.555570, -0.534998, -0.514103, -0.492898, -0.471397, -0.449611, -0.427555, -0.405241,
    -0.382683, -0.359895, -0.336890, -0.313682, -0.290285, -0.266713, -0.242980, -0.219101,
    -0.195090, -0.170962, -0.146730, -0.122411, -0.098017, -0.073565, -0.049068, -0.024541
};
float saxophoneTable[256] = {
    0.000000, 0.046566, 0.092965, 0.139041, 0.184643, 0.229625, 0.273845, 0.317170,
    0.359473, 0.400635, 0.440545, 0.479105, 0.516221, 0.551810, 0.585798, 0.618121,
    0.648723, 0.677555, 0.704577, 0.729759, 0.753079, 0.774526, 0.794097, 0.811798,
    0.827645, 0.841666, 0.853895, 0.864374, 0.873152, 0.880286, 0.885839, 0.889879,
    0.892482, 0.893727, 0.893702, 0.892498, 0.890213, 0.886947, 0.882806, 0.877897,
    0.872330, 0.866218, 0.859676, 0.852818, 0.845761, 0.838620, 0.831512, 0.824550,
    0.817847, 0.811513, 0.805654, 0.800372, 0.795764, 0.791920, 0.788923, 0.786849,
    0.785766, 0.785731, 0.786791, 0.788982, 0.792328, 0.796842, 0.802525, 0.809368,
    0.817351, 0.826444, 0.836604, 0.847778, 0.859904, 0.872910, 0.886719, 0.901246,
    0.916398, 0.932078, 0.948182, 0.964604, 0.981235, 0.997964, 1.014683, 1.031286,
    1.047667, 1.063728, 1.079373, 1.094512, 1.109062, 1.122946, 1.136096, 1.148449,
    1.159955, 1.170567, 1.180247, 1.188966, 1.196704, 1.203447, 1.209191, 1.213936,
    1.217692, 1.220474, 1.222306, 1.223218, 1.223245, 1.222429, 1.220817, 1.218460,
    1.215412, 1.211732, 1.207480, 1.202720, 1.197518, 1.191940, 1.186056, 1.179934,
    1.173644, 1.167255, 1.160835, 1.154450, 1.148165, 1.142040, 1.136133, 1.130499,
    1.125189, 1.120248, 1.115718, 1.111635, 1.108031, 1.104932, 1.102360, 1.100329,
    1.098850, 1.097924, 1.097550, 1.097719, 1.098415, 1.099615, 1.101292, 1.103411,
    1.105937, 1.108827, 1.112036, 1.115514, 1.119210, 1.123067, 1.127029, 1.131038,
    1.135035, 1.138960, 1.142754, 1.146360, 1.149721, 1.152786, 1.155506, 1.157837,
    1.159735, 1.161165, 1.162092, 1.162492, 1.162341, 1.161627, 1.160338, 1.158473,
    1.156034, 1.153032, 1.149481, 1.145406, 1.140830, 1.135785, 1.130307, 1.124434,
    1.118209, 1.111678, 1.104888, 1.097891, 1.090738, 1.083484, 1.076183, 1.068891,
    1.061662, 1.054551, 1.047611, 1.040894, 1.034449, 1.028325, 1.022564, 1.017210,
    1.012298, 1.007865, 1.003938, 1.000544, 0.997704, 0.995436, 0.993753, 0.992665,
    0.992176, 0.992288, 0.992998, 0.994298, 0.996177, 0.998619, 1.001604, 1.005110,
    1.009108, 1.013569, 1.018458, 1.023741, 1.029378, 1.035330, 1.041553, 1.048004,
    1.054637, 1.061406, 1.068263, 1.075160, 1.082051, 1.088888, 1.095625, 1.102218,
    1.108624, 1.114802, 1.120713, 1.126321, 1.131593, 1.136499, 1.141012, 1.145108,
    1.148765, 1.151965, 1.154692, 1.156936, 1.158687, 1.159939, 1.160689, 1.160937,
    1.160687
};
float fluteTable[256] = {
    0.000000, 0.047454, 0.094756, 0.141754, 0.188295, 0.234231, 0.279421, 0.323729,
    0.367024, 0.409184, 0.450094, 0.489646, 0.527739, 0.564281, 0.599185, 0.632377,
    0.663788, 0.693359, 0.721039, 0.746784, 0.770562, 0.792348, 0.812127, 0.829894,
    0.845653, 0.859415, 0.871202, 0.881043, 0.888976, 0.895045, 0.899304, 0.901811,
    0.902632, 0.901840, 0.899512, 0.895731, 0.890587, 0.884173, 0.876586, 0.867926,
    0.858298, 0.847807, 0.836563, 0.824677, 0.812262, 0.799432, 0.786304, 0.772992,
    0.759613, 0.746283, 0.733115, 0.720222, 0.707713, 0.695696, 0.684274, 0.673548,
    0.663612, 0.654558, 0.646470, 0.639428, 0.633505, 0.628765, 0.625266, 0.623058,
    0.622181, 0.622667, 0.624541, 0.627819, 0.632510, 0.638613, 0.646118, 0.655008,
    0.665254, 0.676821, 0.689665, 0.703735, 0.718972, 0.735310, 0.752673, 0.770978,
    0.790134, 0.810042, 0.830596, 0.851683, 0.873185, 0.894978, 0.916934, 0.938919,
    0.960798, 0.982432, 1.003682, 1.024410, 1.044477, 1.063751, 1.082100, 1.099397,
    1.115520, 1.130354, 1.143792, 1.155735, 1.166096, 1.174798, 1.181773, 1.186967,
    1.190336, 1.191849, 1.191486, 1.189243, 1.185127, 1.179161, 1.171380, 1.161832,
    1.150576, 1.137681, 1.123228, 1.107307, 1.090019, 1.071473, 1.051785, 1.031081,
    1.009489, 0.987145, 0.964189, 0.940764, 0.917017, 0.893097, 0.869153, 0.845335,
    0.821790, 0.798666, 0.776105, 0.754248, 0.733231, 0.713182, 0.694226, 0.676480,
    0.660052, 0.645039, 0.631528, 0.619596, 0.609311, 0.600727, 0.593887, 0.588821,
    0.585545, 0.584064, 0.584369, 0.586441, 0.590249, 0.595750, 0.602891, 0.611607,
    0.621820, 0.633444, 0.646379, 0.660514, 0.675728, 0.691891, 0.708865, 0.726502,
    0.744648, 0.763145, 0.781830, 0.800539, 0.819106, 0.837365, 0.855153, 0.872309,
    0.888678, 0.904111, 0.918465, 0.931607, 0.943416, 0.953783, 0.962612, 0.969821,
    0.975343, 0.979130, 0.981149, 0.981388, 0.979847, 0.976546, 0.971520, 0.964823,
    0.956524, 0.946706, 0.935464, 0.922907, 0.909154, 0.894336, 0.878593, 0.862072,
    0.844928, 0.827321, 0.809414, 0.791373, 0.773366, 0.755561, 0.738125, 0.721222,
    0.705013, 0.689655, 0.675299, 0.662089, 0.650163, 0.639648, 0.630662, 0.623312,
    0.617691, 0.613882, 0.611953, 0.611960, 0.613947, 0.617944, 0.623966, 0.632016,
    0.642083, 0.654143, 0.668158, 0.684076, 0.701833, 0.721351, 0.742537, 0.765288,
    0.789487, 0.815007, 0.841709, 0.869446, 0.898062, 0.927392, 0.957267, 0.987511,
    1.017945, 1.048385, 1.078648, 1.108548, 1.137903, 1.166532, 1.194262, 1.220923,
    1.246354, 1.270402, 1.292924, 1.313788, 1.332876, 1.350083, 1.365318, 1.378505,
    1.389584, 1.398511, 1.405259, 1.409820, 1.412203, 1.412437
};
float bellTable[256] = {
    0.000000, 0.024541, 0.049068, 0.073565, 0.098017, 0.122411, 0.146730, 0.170962,
    0.195090, 0.219101, 0.242980, 0.266713, 0.290285, 0.313682, 0.336890, 0.359895,
    0.382683, 0.405241, 0.427555, 0.449611, 0.471397, 0.492898, 0.514103, 0.534998,
    0.555570, 0.575808, 0.595699, 0.615232, 0.634393, 0.653173, 0.671559, 0.689541,
    0.707107, 0.724247, 0.740951, 0.757209, 0.773010, 0.788346, 0.803208, 0.817585,
    0.831470, 0.844854, 0.857729, 0.870087, 0.881921, 0.893224, 0.903989, 0.914210,
    0.923880, 0.932993, 0.941544, 0.949528, 0.956940, 0.963776, 0.970031, 0.975702,
    0.980785, 0.985278, 0.989177, 0.992480, 0.995185, 0.997290, 0.998795, 0.999699,
    1.000000, 0.999699, 0.998795, 0.997290, 0.995185, 0.992480, 0.989177, 0.985278,
    0.980785, 0.975702, 0.970031, 0.963776, 0.956940, 0.949528, 0.941544, 0.932993,
    0.923880, 0.914210, 0.903989, 0.893224, 0.881921, 0.870087, 0.857729, 0.844854,
    0.831470, 0.817585, 0.803208, 0.788346, 0.773010, 0.757209, 0.740951, 0.724247,
    0.707107, 0.689541, 0.671559, 0.653173, 0.634393, 0.615232, 0.595699, 0.575808,
    0.555570, 0.534998, 0.514103, 0.492898, 0.471397, 0.449611, 0.427555, 0.405241,
    0.382683, 0.359895, 0.336890, 0.313682, 0.290285, 0.266713, 0.242980, 0.219101,
    0.195090, 0.170962, 0.146730, 0.122411, 0.098017, 0.073565, 0.049068, 0.024541,
    0.000000, -0.024541, -0.049068, -0.073565, -0.098017, -0.122411, -0.146730, -0.170962,
    -0.195090, -0.219101, -0.242980, -0.266713, -0.290285, -0.313682, -0.336890, -0.359895,
    -0.382683, -0.405241, -0.427555, -0.449611, -0.471397, -0.492898, -0.514103, -0.534998,
    -0.555570, -0.575808, -0.595699, -0.615232, -0.634393, -0.653173, -0.671559, -0.689541,
    -0.707107, -0.724247, -0.740951, -0.757209, -0.773010, -0.788346, -0.803208, -0.817585,
    -0.831470, -0.844854, -0.857729, -0.870087, -0.881921, -0.893224, -0.903989, -0.914210,
    -0.923880, -0.932993, -0.941544, -0.949528, -0.956940, -0.963776, -0.970031, -0.975702,
    -0.980785, -0.985278, -0.989177, -0.992480, -0.995185, -0.997290, -0.998795, -0.999699,
    -1.000000, -0.999699, -0.998795, -0.997290, -0.995185, -0.992480, -0.989177, -0.985278,
    -0.980785, -0.975702, -0.970031, -0.963776, -0.956940, -0.949528, -0.941544, -0.932993,
    -0.923880, -0.914210, -0.903989, -0.893224, -0.881921, -0.870087, -0.857729, -0.844854,
    -0.831470, -0.817585, -0.803208, -0.788346, -0.773010, -0.757209, -0.740951, -0.724247,
    -0.707107, -0.689541, -0.671559, -0.653173, -0.634393, -0.615232, -0.595699, -0.575808,
    -0.555570, -0.534998, -0.514103, -0.492898, -0.471397, -0.449611, -0.427555, -0.405241,
    -0.382683, -0.359895, -0.336890, -0.313682, -0.290285, -0.266713, -0.242980, -0.219101,
    -0.195090, -0.170962, -0.146730, -0.122411, -0.098017, -0.073565, -0.049068, -0.024541
};
float sineTable[256] = {
    0.000000, 0.024541, 0.049068, 0.073565, 0.098017, 0.122411, 0.146730, 0.170962,
    0.195090, 0.219101, 0.242980, 0.266713, 0.290285, 0.313682, 0.336890, 0.359895,
    0.382683, 0.405241, 0.427555, 0.449611, 0.471397, 0.492898, 0.514103, 0.534998,
    0.555570, 0.575808, 0.595699, 0.615232, 0.634393, 0.653173, 0.671559, 0.689541,
    0.707107, 0.724247, 0.740951, 0.757209, 0.773010, 0.788346, 0.803208, 0.817585,
    0.831470, 0.844854, 0.857729, 0.870087, 0.881921, 0.893224, 0.903989, 0.914210,
    0.923880, 0.932993, 0.941544, 0.949528, 0.956940, 0.963776, 0.970031, 0.975702,
    0.980785, 0.985278, 0.989177, 0.992480, 0.995185, 0.997290, 0.998795, 0.999699,
    1.000000, 0.999699, 0.998795, 0.997290, 0.995185, 0.992480, 0.989177, 0.985278,
    0.980785, 0.975702, 0.970031, 0.963776, 0.956940, 0.949528, 0.941544, 0.932993,
    0.923880, 0.914210, 0.903989, 0.893224, 0.881921, 0.870087, 0.857729, 0.844854,
    0.831470, 0.817585, 0.803208, 0.788346, 0.773010, 0.757209, 0.740951, 0.724247,
    0.707107, 0.689541, 0.671559, 0.653173, 0.634393, 0.615232, 0.595699, 0.575808,
    0.555570, 0.534998, 0.514103, 0.492898, 0.471397, 0.449611, 0.427555, 0.405241,
    0.382683, 0.359895, 0.336890, 0.313682, 0.290285, 0.266713, 0.242980, 0.219101,
    0.195090, 0.170962, 0.146730, 0.122411, 0.098017, 0.073565, 0.049068, 0.024541,
    0.000000, -0.024541, -0.049068, -0.073565, -0.098017, -0.122411, -0.146730, -0.170962,
    -0.195090, -0.219101, -0.242980, -0.266713, -0.290285, -0.313682, -0.336890, -0.359895,
    -0.382683, -0.405241, -0.427555, -0.449611, -0.471397, -0.492898, -0.514103, -0.534998,
    -0.555570, -0.575808, -0.595699, -0.615232, -0.634393, -0.653173, -0.671559, -0.689541,
    -0.707107, -0.724247, -0.740951, -0.757209, -0.773010, -0.788346, -0.803208, -0.817585,
    -0.831470, -0.844854, -0.857729, -0.870087, -0.881921, -0.893224, -0.903989, -0.914210,
    -0.923880, -0.932993, -0.941544, -0.949528, -0.956940, -0.963776, -0.970031, -0.975702,
    -0.980785, -0.985278, -0.989177, -0.992480, -0.995185, -0.997290, -0.998795, -0.999699,
    -1.000000, -0.999699, -0.998795, -0.997290, -0.995185, -0.992480, -0.989177, -0.985278,
    -0.980785, -0.975702, -0.970031, -0.963776, -0.956940, -0.949528, -0.941544, -0.932993,
    -0.923880, -0.914210, -0.903989, -0.893224, -0.881921, -0.870087, -0.857729, -0.844854,
    -0.831470, -0.817585, -0.803208, -0.788346, -0.773010, -0.757209, -0.740951, -0.724247,
    -0.707107, -0.689541, -0.671559, -0.653173, -0.634393, -0.615232, -0.595699, -0.575808,
    -0.555570, -0.534998, -0.514103, -0.492898, -0.471397, -0.449611, -0.427555, -0.405241,
    -0.382683, -0.359895, -0.336890, -0.313682, -0.290285, -0.266713, -0.242980, -0.219101,
    -0.195090, -0.170962, -0.146730, -0.122411, -0.098017, -0.073565, -0.049068, -0.024541
};
float triangleTable[256] = {
    0.000000, 0.007812, 0.015625, 0.023438, 0.031250, 0.039062, 0.046875, 0.054688,
    0.062500, 0.070312, 0.078125, 0.085938, 0.093750, 0.101562, 0.109375, 0.117188,
    0.125000, 0.132812, 0.140625, 0.148438, 0.156250, 0.164062, 0.171875, 0.179688,
    0.187500, 0.195312, 0.203125, 0.210938, 0.218750, 0.226562, 0.234375, 0.242188,
    0.250000, 0.257812, 0.265625, 0.273438, 0.281250, 0.289062, 0.296875, 0.304688,
    0.312500, 0.320312, 0.328125, 0.335938, 0.343750, 0.351562, 0.359375, 0.367188,
    0.375000, 0.382812, 0.390625, 0.398438, 0.406250, 0.414062, 0.421875, 0.429688,
    0.437500, 0.445312, 0.453125, 0.460938, 0.468750, 0.476562, 0.484375, 0.492188,
    0.500000, 0.507812, 0.515625, 0.523438, 0.531250, 0.539062, 0.546875, 0.554688,
    0.562500, 0.570312, 0.578125, 0.585938, 0.593750, 0.601562, 0.609375, 0.617188,
    0.625000, 0.632812, 0.640625, 0.648438, 0.656250, 0.664062, 0.671875, 0.679688,
    0.687500, 0.695312, 0.703125, 0.710938, 0.718750, 0.726562, 0.734375, 0.742188,
    0.750000, 0.757812, 0.765625, 0.773438, 0.781250, 0.789062, 0.796875, 0.804688,
    0.812500, 0.820312, 0.828125, 0.835938, 0.843750, 0.851562, 0.859375, 0.867188,
    0.875000, 0.882812, 0.890625, 0.898438, 0.906250, 0.914062, 0.921875, 0.929688,
    0.937500, 0.945312, 0.953125, 0.960938, 0.968750, 0.976562, 0.984375, 0.992188,
    1.000000, 0.992188, 0.984375, 0.976562, 0.968750, 0.960938, 0.953125, 0.945312,
    0.937500, 0.929688, 0.921875, 0.914062, 0.906250, 0.898438, 0.890625, 0.882812,
    0.875000, 0.867188, 0.859375, 0.851562, 0.843750, 0.835938, 0.828125, 0.820312,
    0.812500, 0.804688, 0.796875, 0.789062, 0.781250, 0.773438, 0.765625, 0.757812,
    0.750000, 0.742188, 0.734375, 0.726562, 0.718750, 0.710938, 0.703125, 0.695312,
    0.687500, 0.679688, 0.671875, 0.664062, 0.656250, 0.648438, 0.640625, 0.632812,
    0.625000, 0.617188, 0.609375, 0.601562, 0.593750, 0.585938, 0.578125, 0.570312,
    0.562500, 0.554688, 0.546875, 0.539062, 0.531250, 0.523438, 0.515625, 0.507812,
    0.500000, 0.492188, 0.484375, 0.476562, 0.468750, 0.460938, 0.453125, 0.445312,
    0.437500, 0.429688, 0.421875, 0.414062, 0.406250, 0.398438, 0.390625, 0.382812,
    0.375000, 0.367188, 0.359375, 0.351562, 0.343750, 0.335938, 0.328125, 0.320312,
    0.312500, 0.304688, 0.296875, 0.289062, 0.281250, 0.273438, 0.265625, 0.257812,
    0.250000, 0.242188, 0.234375, 0.226562, 0.218750, 0.210938, 0.203125, 0.195312,
    0.187500, 0.179688, 0.171875, 0.164062, 0.156250, 0.148438, 0.140625, 0.132812,
    0.125000, 0.117188, 0.109375, 0.101562, 0.093750, 0.085938, 0.078125, 0.070312,
    0.062500, 0.054688, 0.046875, 0.039062, 0.031250, 0.023438, 0.015625, 0.007812
};
float dingTable[256] = {
    0.000000, 0.024541, 0.049068, 0.073565, 0.098017, 0.122411, 0.146730, 0.170962,
    0.195090, 0.219101, 0.242980, 0.266713, 0.290285, 0.313682, 0.336890, 0.359895,
    0.382683, 0.405241, 0.427555, 0.449611, 0.471397, 0.492898, 0.514103, 0.534998,
    0.555570, 0.575808, 0.595699, 0.615232, 0.634393, 0.653173, 0.671559, 0.689541,
    0.707107, 0.724247, 0.740951, 0.757209, 0.773010, 0.788346, 0.803208, 0.817585,
    0.831470, 0.844854, 0.857729, 0.870087, 0.881921, 0.893224, 0.903989, 0.914210,
    0.923880, 0.932993, 0.941544, 0.949528, 0.956940, 0.963776, 0.970031, 0.975702,
    0.980785, 0.985278, 0.989177, 0.992480, 0.995185, 0.997290, 0.998795, 0.999699,
    1.000000, 0.999699, 0.998795, 0.997290, 0.995185, 0.992480, 0.989177, 0.985278,
    0.980785, 0.975702, 0.970031, 0.963776, 0.956940, 0.949528, 0.941544, 0.932993,
    0.923880, 0.914210, 0.903989, 0.893224, 0.881921, 0.870087, 0.857729, 0.844854,
    0.831470, 0.817585, 0.803208, 0.788346, 0.773010, 0.757209, 0.740951, 0.724247,
    0.707107, 0.689541, 0.671559, 0.653173, 0.634393, 0.615232, 0.595699, 0.575808,
    0.555570, 0.534998, 0.514103, 0.492898, 0.471397, 0.449611, 0.427555, 0.405241,
    0.382683, 0.359895, 0.336890, 0.313682, 0.290285, 0.266713, 0.242980, 0.219101,
    0.195090, 0.170962, 0.146730, 0.122411, 0.098017, 0.073565, 0.049068, 0.024541,
    0.000000, -0.024541, -0.049068, -0.073565, -0.098017, -0.122411, -0.146730, -0.170962,
    -0.195090, -0.219101, -0.242980, -0.266713, -0.290285, -0.313682, -0.336890, -0.359895,
    -0.382683, -0.405241, -0.427555, -0.449611, -0.471397, -0.492898, -0.514103, -0.534998,
    -0.555570, -0.575808, -0.595699, -0.615232, -0.634393, -0.653173, -0.671559, -0.689541,
    -0.707107, -0.724247, -0.740951, -0.757209, -0.773010, -0.788346, -0.803208, -0.817585,
    -0.831470, -0.844854, -0.857729, -0.870087, -0.881921, -0.893224, -0.903989, -0.914210,
    -0.923880, -0.932993, -0.941544, -0.949528, -0.956940, -0.963776, -0.970031, -0.975702,
    -0.980785, -0.985278, -0.989177, -0.992480, -0.995185, -0.997290, -0.998795, -0.999699,
    -1.000000, -0.999699, -0.998795, -0.997290, -0.995185, -0.992480, -0.989177, -0.985278,
    -0.980785, -0.975702, -0.970031, -0.963776, -0.956940, -0.949528, -0.941544, -0.932993,
    -0.923880, -0.914210, -0.903989, -0.893224, -0.881921, -0.870087, -0.857729, -0.844854,
    -0.831470, -0.817585, -0.803208, -0.788346, -0.773010, -0.757209, -0.740951, -0.724247,
    -0.707107, -0.689541, -0.671559, -0.653173, -0.634393, -0.615232, -0.595699, -0.575808,
    -0.555570, -0.534998, -0.514103, -0.492898, -0.471397, -0.449611, -0.427555, -0.405241,
    -0.382683, -0.359895, -0.336890, -0.313682, -0.290285, -0.266713, -0.242980, -0.219101,
    -0.195090, -0.170962, -0.146730, -0.122411, -0.098017, -0.073565, -0.049068, -0.024541
};

float dongTable[256] = {
    0.000000, 0.024541, 0.049068, 0.073565, 0.098017, 0.122411, 0.146730, 0.170962,
    0.195090, 0.219101, 0.242980, 0.266713, 0.290285, 0.313682, 0.336890, 0.359895,
    0.382683, 0.405241, 0.427555, 0.449611, 0.471397, 0.492898, 0.514103, 0.534998,
    0.555570, 0.575808, 0.595699, 0.615232, 0.634393, 0.653173, 0.671559, 0.689541,
    0.707107, 0.724247, 0.740951, 0.757209, 0.773010, 0.788346, 0.803208, 0.817585,
    0.831470, 0.844854, 0.857729, 0.870087, 0.881921, 0.893224, 0.903989, 0.914210,
    0.923880, 0.932993, 0.941544, 0.949528, 0.956940, 0.963776, 0.970031, 0.975702,
    0.980785, 0.512861, 0.493368, 0.469628, 0.441961, 0.410715, 0.376281, 0.339093,
    0.299625, 0.258392, 0.215942, 0.172854, 0.129734, 0.087201, 0.045884, 0.006405,
    -0.030635, -0.064659, -0.095220, -0.122004, -0.144737, -0.163189, -0.177176, -0.186564,
    -0.191270, -0.191270, -0.186564, -0.177176, -0.163189, -0.144737, -0.122004, -0.095220,
    -0.064659, -0.030635, 0.006405, 0.045884, 0.087201, 0.129734, 0.172854, 0.215942,
    0.258392, 0.299625, 0.339093, 0.376281, 0.410715, 0.441961, 0.469628, 0.493368,
    0.512861, 0.527726, 0.537738, 0.542738, 0.542632, 0.537394, 0.527068, 0.511770,
    0.491691, 0.467091, 0.438301, 0.405720, 0.369813, 0.331104, 0.290173, 0.247649,
    0.204200, 0.160532, 0.117375, 0.075475, 0.035585, -0.001558, -0.035244, -0.064803,
    -0.089618, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000

};
float squareTable[256] = {
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0
};



#endif  // PIN_DEFINITIONS_H
