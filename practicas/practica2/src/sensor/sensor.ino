// *=> imports
#include <Wire.h>
#include <SSD1306AsciiWire.h>

#include "oled_commands.h"
#include "uss_commands.h"

// *=> consts

#define MAX_SERIAL_TIME 200
#define USS_QUANTITY 2

// *=> vars
byte segments[3];
uint8_t idx;
uint32_t time;

// TODO
// 
uint32_t oled_period = 0;
uint32_t oled_time = 0;

byte uss_addr[] = { byte(0xE0) >> 1,(0xE2) >> 1 };
uint32_t uss_time[] = { 0, 0 };
uint32_t uss_period[] = { 1000, 1000 };
uint16_t uss_delay[] = { 100, 100 };
uint8_t uss_measureUnit[] = { 0, 0 };

uint16_t uss_measure_data[] = { 0, 0 };
uint16_t uss_autotune_data[] = { 0, 0 };



// *=> functions headers

bool receiveSegments();
bool receiveSegments();
void sensorController();
void oledController();

// *=> main

void setup() {

    Serial.begin(9600);
    while (!Serial) {}

    Serial1.begin(9600);
    while (!Serial1) {}

    Wire.begin();
    delay(100);

    oled_init(0);
    oledController();
}

void loop() {

    receiveSegments() && playSegment();

    //  for (int i = 0; i < USS_QUANTITY; i++) if ((millis() - uss_time[i]) >= uss_period[i]) sensorController(i);
}


bool receiveSegments() {
    for (idx = 0; idx < 3; idx++) segments[idx] = 0;
    idx = 0;
    if (Serial1.available()) {
        time = millis();
        while ((millis() - time) < MAX_SERIAL_TIME) {
            if (!Serial1.available()) { delay(5); continue; }
            if (idx >= 3) return false;

            segments[idx++] = Serial1.read();
        }

        return true;
    }

    return false;
}


// TODO
bool playSegment() {
    Serial.println("");
    Serial.println(String(segments[0], BIN));
    Serial.println(String(uint16_t((segments[1] << 8) | segments[2])));

    /*
    switch (segments[0] & 0b11) {
    case 0:
    case 1:
    case 2:
    case 3:
    }
    */

    return false;
}

// TODO
void sensorController(uint8_t uss_index) {

    uss_measure(uss_addr[uss_index],
        uss_measureUnit[uss_index] == 0 ? REAL_RANGING_MODE_INCHES : uss_measureUnit[uss_index] == 1 ? REAL_RANGING_MODE_CMS : REAL_RANGING_MODE_USECS,
        uss_delay[uss_index], &(uss_measure_data[uss_index]), &(uss_autotune_data[uss_index]));

    uss_time[uss_index] = millis();
    oledController();
}


void oledController() {

    oled.clear();

    oled.println("Sensor 0: " + String(uss_measure_data[0]));
    oled.println(String(uss_measureUnit[0] == 0 ? "inc" : uss_measureUnit[0] == 1 ? " cm" : " ms") + " min= " + String(uss_autotune_data[0]));
    oled.println("Sensor 1: " + String(uss_measure_data[1]));
    oled.println(String(uss_measureUnit[0] == 0 ? "inc" : uss_measureUnit[0] == 1 ? " cm" : " ms") + " min= " + String(uss_autotune_data[1]));
}