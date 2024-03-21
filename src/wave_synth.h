#include <math.h>
#include <stdlib.h>
#include <pin_definitions.h>

#define SAMPLE_RATE 22000
#define AMPLITUDE 0.5
#define TABLE_SIZE 256
#define PI M_PI


// ADSR envelope parameters
float attack_time = 0.1;
float decay_time = 0.2;
float sustain_level = 0.6;
float release_time = 0.4;

// Filter parameters
float filter_cutoff = 0.5;
float filter_resonance = 0.2;
float filter_env_amount = 2;

// LFO parameters for pitch modulation
float lfo_frequency = 5.0;
float lfo_depth = 0.01;

// Noise parameters
float noise_level = 0.1;
float noise_filter_cutoff = 0.8;



















float dt=1.0f / SAMPLE_RATE;
float prevcutoffFreq=500;
float rc=1.0f / (2.0f * M_PI * 500);
float alpha= dt / (rc + dt);


float lowPassFilter(float cur, float prev, float cutoffFreq) {
    
    if (prevcutoffFreq!=cutoffFreq){
        rc = 1.0f / (2.0f * M_PI * cutoffFreq);
        alpha = dt / (rc + dt);
    }

    prevcutoffFreq=cutoffFreq;
    // float 


    return  prev + alpha * (cur - prev);
        // prev = buffer[i];
    // }
}



float low_pass_filter(float input, float cutoff, float resonance, float* state1, float* state2) {
    float output = input * cutoff + *state1 * (1.0 - cutoff);
    *state1 = output * cutoff + *state2 * (1.0 - cutoff);
    *state2 = output;
    output += (output - *state2) * resonance;
    return output;
}




float generateTriangleWave( float frequency ){
    float amplitude=0.5;
    float period = SAMPLE_RATE / frequency;
    float increment = 4.0f * amplitude / period;
    float value = -amplitude;

    if (value >= amplitude) {
        increment = -increment;
    } else if (value <= -amplitude) {
        increment = -increment;
    }

    value += increment;
    return value;
}

float generateTriangleWaveValue(float frequency, float* phase) {
    *phase += frequency / SAMPLE_RATE;
    if (*phase > 1.0f) {
        *phase -= 1.0f;
    }

    float value = 2.0f * AMPLITUDE * (fabs(2.0f * *phase - 1.0f) - 0.5f);
    return value;
}
float bhaskaraSin(float x) {
    float numerator = 16 * x * (PI - x);
    float denominator = 5 * PI * PI - 4 * x * (PI - x);
    return numerator / denominator;// faster sin but actually slower
}
float generateSin( float sinPhase, float* phase){
    float testsinAcc=*phase;
    testsinAcc+=sinPhase;
    if (testsinAcc>=M_PI){
        testsinAcc-=M_PI;
    }
    *phase=testsinAcc;
    return sin(testsinAcc);
}


float getSample(float phaseIcre, float* phaseAcc, float table[]) {
    *phaseAcc+=phaseIcre;
    if (int(*phaseAcc*TABLE_SIZE)>TABLE_SIZE){
        *phaseAcc-=1;
    }
    int index = (int)(*phaseAcc * TABLE_SIZE);
    index = index % TABLE_SIZE;
    return table[index];
}
float LFOAcc=0;
float prevLFOfreq=20;
float lfoPhase=20*M_PI*2/SAMPLE_RATE;
float generateLFO(int reduceVal,float lfoFreq){
    // float lfoFreq=10.0;
    if(lfoFreq!=prevLFOfreq){
        lfoPhase=lfoFreq*M_PI*2/SAMPLE_RATE;
        
    }
    prevLFOfreq=lfoFreq;
    

    float amp=getSample(lfoPhase,&LFOAcc,sineTable)/reduceVal;
    return amp;
}
// float sinTable[256];

// void generateSinLUT(){
//     float step=(1/256)*2*M_PI;
//     for (int i=0; i<TABLE_SIZE;i++){
//         sinTable[i]=sin(step*i);
//     }
// }
int calcFade(int pressedCount, int decaytime, int decayspeed){
    // int press=3;
    // int decay=4;
    if (pressedCount<decaytime){
        return 0;
    }
    else{
        return int( (pressedCount-decaytime)/decayspeed);
    }

}
float calcSawtoothAmp(float *phaseAcc,int volume, int i){
    // uint32_t stepSize=__atomic_load_n(&stepSizes[i%12],__ATOMIC_RELAXED); 
    float stepsize=notePhases[i];


    *phaseAcc+=stepsize;
    if (*phaseAcc>1){
        *phaseAcc-=1;
    }
    float amp=*phaseAcc;
    // uint32_t Vout = (*phaseAcc >> 24) - 128;
    // Vout = (Vout+128) >> (8 - volume)) ;

    return amp;
}
u_int32_t calcNoProcessSawtoothVout(u_int32_t phaseAcc,int volume, int tune){
    
    uint32_t Vout = (phaseAcc >> 24) - 128;

    // int v=calcEnvelope(notes.notes[i].pressedCount);
    int volshift=8 - volume;

    Vout = ((Vout+128) >> (volshift)) ;
    return Vout;
}



// u_int32_t calcOtherVout(float Amp,int volume, int i){
//     uint32_t Vout = static_cast<uint32_t>(Amp *127) - 128;
//     int v=calcDecay(notes.notes[i].pressedCount);
//     int volshift=8 - volume+v;
//     if (volshift>=0){

//     Vout = ((Vout+128) >> (volshift)) ;}
//     else {Vout = ((Vout+128) << -(volshift)) ;}

//     return Vout;
// }

u_int32_t calcNoEnvelopeVout(float Amp,int volume){
    uint32_t Vout = static_cast<uint32_t>(Amp *127) - 128;
    int volshift=8 - volume;
    if (volshift>=0){

    Vout = ((Vout+128) >> (volshift)) ;}
    else {Vout = ((Vout+128) << -(volshift)) ;}

    return Vout;
}

u_int32_t calcVout(float Amp,int volume, int vshift){
    uint32_t Vout = static_cast<uint32_t>(Amp *255) - 128;
    int volshift=8 - volume+vshift;
    if (volshift>=0){

    Vout = ((Vout+128) >> (volshift)) ;}
    else {Vout = ((Vout+128) << -(volshift)) ;}

    return Vout;
}



int adsrGeneral(int pressedCount){
    int attack=settings.adsr.attack;
    int decay=settings.adsr.decay+attack;
    int sustain=settings.adsr.sustain+decay;
    if (pressedCount<attack && pressedCount>0){
        return pressedCount-int(attack/2);
    }
    else if (pressedCount>= attack && decay>=pressedCount){
        return int( (pressedCount-attack));
    }
    else if(pressedCount>decay && pressedCount<sustain){
        return int( (decay-attack));
    }

    else{
        return (decay-attack)+(pressedCount-sustain)/settings.fade.fadeSpeed;
    }
}

int adsrHorn(int pressedCount){
    int lowtime=3; 
    // int hightime;
    int lowval=2;
    int highval=-2;
    int loopduration=20;
    static int countlow=0;
    static int counthigh=0;
    if ((pressedCount%loopduration)<=lowtime){
        counthigh=0;
        if (countlow<lowval){
            
            countlow+=1;
        }
        return countlow;
    }
    else{
        countlow=0;
        if (counthigh>highval){
            counthigh-=1;
        }
        return counthigh;
    }
    
}
u_int32_t calcPianoVout(float Amp,int volume, int i){
    // Amp+=generateLFO(2);
    uint32_t Vout = static_cast<uint32_t>(Amp *127) - 128;
    int v=adsrGeneral(notes.notes[i].pressedCount);
    // int v=adsrHorn(notes.notes[i].pressedCount);
    int volshift=8 - volume+v;
    if (volshift>=0){

    Vout = ((Vout+128) >> (volshift)) ;}
    else {Vout = ((Vout+128) << -(volshift)) ;}

    return Vout;
}
u_int32_t calcHornVout(float Amp,int volume, int i){
    // Amp+=generateLFO(2);
    uint32_t Vout = static_cast<uint32_t>(Amp *127) - 128;
    int v=adsrHorn(notes.notes[i].pressedCount);
    // int v=adsrHorn(notes.notes[i].pressedCount);
    int volshift=8 - volume+v;
    if (volshift>=0){

    Vout = ((Vout+128) >> (volshift)) ;}
    else {Vout = ((Vout+128) << -(volshift)) ;}

    return Vout;
}


void presssedTimeCount(){
    for (int i=0;i<96;i++){
        bool isactive=__atomic_load_n(&notes.notes[i].active,__ATOMIC_RELAXED);
        if (isactive){
            notes.notes[i].pressedCount+=1;
        }
        else{
            notes.notes[i].pressedCount=0;
        }
    }
}

u_int32_t addEffects(float amp, int volume, int i){
    
    int vshift=0;

    if (settings.fade.on){
        int pc=notes.notes[i].pressedCount;
        vshift=calcFade(pc,settings.fade.sustainTime,settings.fade.fadeSpeed);
    }
    else if (settings.adsr.on){
        int pc=notes.notes[i].pressedCount;
        vshift=adsrGeneral(pc);
    }
    u_int32_t Vout=calcVout(amp, volume,vshift);
    return Vout;
}
u_int32_t addLFO(float amp,int volume){
    if (settings.lfo.on){
        amp=generateLFO(settings.lfo.reduceLFOVolume,settings.lfo.freq);
        
        u_int32_t Vout=calcVout(amp, volume,0);
        return Vout;
    }
    else{
        return 0;
    }


}
float addLPF(float amp,float *prevamp){
    if (settings.lowpass.on){
        amp=lowPassFilter(amp, *prevamp,settings.lowpass.freq);
        *prevamp=amp;
       
    }
    return amp;

}
