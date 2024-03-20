#include "pin_definitions.h"

#ifndef MENU_H
#define MENU_H

std::string convert_bool_2_string(bool value) {
    return value ? "On" : "Off";
}

void met_page(Metronome metronome_inside_page){
    sysState.currentMenu = "Met";
    u8g2.drawStr(10, 7, "Metronome");
    u8g2.drawStr(10, 14, "State: ");
    u8g2.drawStr(50, 14, convert_bool_2_string(metronome_inside_page.on).c_str());
    u8g2.drawStr(10, 21, "Speed: ");
    u8g2.drawStr(50, 21, std::to_string(metronome_inside_page.speed).c_str());
}

void fade_page(Fade fade_inside_page) {
    sysState.currentMenu = "Fade";
    u8g2.drawStr(10, 7, "Fade");
    u8g2.drawStr(10, 14, "State: ");
    u8g2.drawStr(50, 14, convert_bool_2_string(fade_inside_page.on).c_str());
    u8g2.drawStr(10, 21, "sustainTime: ");
    u8g2.drawStr(70, 21, std::to_string(fade_inside_page.sustainTime).c_str());
    u8g2.drawStr(10, 28, "fadeSpeed: ");
    u8g2.drawStr(60, 28, std::to_string(fade_inside_page.fadeSpeed).c_str());

}

void lfo_page(LFO lfo_inside_page) {
    sysState.currentMenu = "LFO";
    u8g2.drawStr(10, 7, "Low Fre Oscillator");
    u8g2.drawStr(10, 14, "State: ");
    u8g2.drawStr(50, 14, convert_bool_2_string(lfo_inside_page.on).c_str());
    u8g2.drawStr(10, 21, "freq:");
    u8g2.drawStr(70, 21, std::to_string(lfo_inside_page.freq).c_str());
    u8g2.drawStr(10, 28, "reduceLFOVolume:");
    u8g2.drawStr(90, 28, std::to_string(lfo_inside_page.reduceLFOVolume).c_str());

}

void adsr_page(ADSR adsr_inside_page) {
    sysState.currentMenu = "ADSR";
    u8g2.drawStr(10, 7, "ADSR Envelope Gen");
    u8g2.drawStr(10, 14, "on? attack dec sustain");
    u8g2.drawStr(10, 21, convert_bool_2_string(adsr_inside_page.on).c_str());
    u8g2.drawStr(40, 21, std::to_string(adsr_inside_page.attack).c_str());
    u8g2.drawStr(70, 21, std::to_string(adsr_inside_page.decay).c_str());
    u8g2.drawStr(100, 21, std::to_string(adsr_inside_page.sustain).c_str());
}

void lpf_page(Lowpass lowpass_inside_page) {
    sysState.currentMenu = "LPF";
    u8g2.drawStr(10, 7, "Low Pass Filter");
    u8g2.drawStr(10, 14, "State: ");
    u8g2.drawStr(50, 14, convert_bool_2_string(lowpass_inside_page.on).c_str());
    u8g2.drawStr(10, 21, "Fre: ");
    u8g2.drawStr(50, 21, std::to_string(lowpass_inside_page.freq).c_str());
}

void empty_page() {
    u8g2.drawStr(10, 10, "Empty");
}

void menu(int option, setting setting_inside_menu) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tr);
    switch(option) {
        case 0: met_page(setting_inside_menu.metronome); break;
        case 1: fade_page(setting_inside_menu.fade); break;
        case 2: lfo_page(setting_inside_menu.lfo); break;
        case 3: adsr_page(setting_inside_menu.adsr); break;
        case 4: lpf_page(setting_inside_menu.lowpass); break;
        case 5: empty_page(); break;
        default: empty_page(); break;
    }
}

void update_menu_settings(std::string option){
    if (option == "Main"){
        settings.volume += sysState.knobValues[3].lastIncrement;
        settings.Tone += sysState.knobValues[2].lastIncrement;
        settings.waveIndex += sysState.knobValues[1].lastIncrement;
        settings.volume  = constrain(settings.volume, 0, 8);
        settings.Tone = constrain(settings.Tone, 0, 8);
        settings.waveIndex = constrain(settings.waveIndex, 0, 8);
    }

    else if (option == "Met"){
      settings.metronome.on = sysState.knobValues[1].clickState;
      settings.metronome.speed += sysState.knobValues[2].lastIncrement;
      settings.metronome.speed = constrain(settings.metronome.speed, -4, 4);
    }

    else if (option == "Fade"){
      settings.fade.on = sysState.knobValues[1].clickState;
      settings.fade.fadeSpeed += sysState.knobValues[3].lastIncrement;
      settings.fade.sustainTime += sysState.knobValues[2].lastIncrement;
      settings.fade.fadeSpeed = constrain(settings.fade.fadeSpeed, 0, 8);
      settings.fade.sustainTime = constrain(settings.fade.sustainTime, 0, 8);
    }

    else if (option == "LFO"){
      settings.lfo.on = sysState.knobValues[1].clickState;
      settings.lfo.freq += sysState.knobValues[2].lastIncrement;
      settings.lfo.reduceLFOVolume += sysState.knobValues[3].lastIncrement;
      settings.lfo.freq = constrain(settings.lfo.freq, 0, 8);
      settings.lfo.reduceLFOVolume = constrain(settings.lfo.reduceLFOVolume, 0, 8);
    }

    else if (option == "ADSR"){
      settings.adsr.on = sysState.knobValues[1].clickState;
      settings.adsr.attack += sysState.knobValues[1].lastIncrement;
      settings.adsr.decay += sysState.knobValues[2].lastIncrement;
      settings.adsr.sustain += sysState.knobValues[3].lastIncrement;
      settings.adsr.attack = constrain(settings.adsr.attack, 0, 8);
      settings.adsr.decay = constrain(settings.adsr.decay, 0, 8);
      settings.adsr.sustain = constrain(settings.adsr.sustain, 0, 8);
    }

    else if (option == "LPF"){
      settings.lowpass.on = sysState.knobValues[1].clickState;
      settings.lowpass.freq += sysState.knobValues[2].lastIncrement;
      settings.lowpass.freq = constrain(settings.lowpass.freq, 0, 8);
    }
}
#endif