#include "pin_definitions.h"

#ifndef MENU_H
#define MENU_H

std::string convert_bool_2_string(bool value) {
    return value ? "On" : "Off";
}

void met_page(){
    u8g2.drawStr(10, 10, "Metronome");
    u8g2.drawStr(10, 20, "State: ");
    u8g2.drawStr(50, 20, convert_bool_2_string(settings.metronome.on).c_str());
    u8g2.drawStr(10, 30, "Speed: ");
    u8g2.drawStr(50, 30, std::to_string(settings.metronome.speed).c_str());
}

void fade_page() {
    u8g2.drawStr(10, 7, "Fade");
    u8g2.drawStr(10, 14, "State: ");
    u8g2.drawStr(50, 14, convert_bool_2_string(settings.fade.on).c_str());
    u8g2.drawStr(10, 21, "sustainTime: ");
    u8g2.drawStr(70, 21, std::to_string(settings.fade.sustainTime).c_str());
    u8g2.drawStr(10, 28, "fadeSpeed: ");
    u8g2.drawStr(60, 28, std::to_string(settings.fade.fadeSpeed).c_str());

}

void lfo_page() {
    u8g2.drawStr(10, 10, "Low Frequency");
    u8g2.drawStr(10, 20, "Oscillator");
}

void adsr_page() {
    u8g2.drawStr(10, 10, "ADSR");
    u8g2.drawStr(10, 20, "Envelope Generator");
}

void lpf_page() {
    u8g2.drawStr(10, 10, "Low Pass Filter");
}

void empty_page() {
    u8g2.drawStr(10, 10, "Empty");
}

void menu(int option) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tr);
    switch(option) {
        case 0: met_page(); break;
        case 1: fade_page(); break;
        case 2: lfo_page(); break;
        case 3: adsr_page(); break;
        case 4: lpf_page(); break;
        case 5: empty_page(); break;
        default: empty_page(); break;
    }
}


#endif