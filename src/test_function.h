#include <math.h>
#include <stdlib.h>
#include <pin_definitions.h>



void setWorstcaseBackCalc(){
    //the worst case senerio for backcalculation thread is when all 48 keys on all 4 keyboards have been pressed
    //this function fills the first 48 index of the notes array, where about the array is filled does not make an impact on the excution time
    for (int i=0;i<48;i++){
        notes.notes[i].active=true;
    }
    // and it will be very time consuming to turn all of the features on, the value of the features do not matter much, we just use their default settings
    settings.adsr.on;// either adsr or fade, adsr does more computation
    settings.lfo.on;
    settings.lowpass.on;
}
// void backCalcTime(){
//     setWorstcaseBackCalc();
//     uint32_t startTime = micros();
//     // delayMicroseconds(3);
//     // Serial.println(micros()-startTime);

    
// 	for (int iter = 0; iter < 32; iter++) {
// 		 backgroundCalcTask(NULL);
//      Serial.println(micros()-startTime);
// 	}
	

// }



// void testSetup(){
//   sysState.knobValues[2].current_knob_value = 4;
//   sysState.knobValues[3].current_knob_value = 6;
//   //Set pin directions
//   generatePhaseLUT();
//   set_pin_directions();
//   set_notes();
//   init_settings();
//   Serial.begin(9600);
//   Serial.println("Serial port initialised");
//   sysState.knobValues[2].current_knob_value = sysState.posId + 3;
//   backCalcTime();
// }