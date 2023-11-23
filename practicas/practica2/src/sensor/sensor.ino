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

byte uss_addr;
byte uss_option;

// TODO
uint32_t oled_period = 0;
uint32_t oled_time = 0;

byte uss_addrs[] = { byte(0xE0) >> 1, byte(0xE2) >> 1 };
uint32_t uss_time[] = { 0, 0 };
uint16_t uss_period[] = { 500, 350 };
uint16_t uss_delay[] = { 50, 80 };
uint8_t uss_measureUnit[] = { 0, 0 };
bool uss_oneShot[] = { false, false };

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

    for (int i = 0; i < USS_QUANTITY; i++)
        if ((uss_period[i] || uss_oneShot[i]) && (millis() - uss_time[i]) >= uss_period[i])
            sensorController(i);
}


bool receiveSegments() {
    for (idx = 0; idx < 3; idx++) segments[idx] = 0;
    idx = 0;
    if (Serial1.available()) {
        time = millis();
        while ((millis() - time) < MAX_SERIAL_TIME) {
            if (!Serial1.available()) { delay(10); continue; }
            if (idx >= 3) return false;

            segments[idx++] = Serial1.read();
        }

        return true;
    }

    return false;
}


// TODO
bool playSegment() {

    uss_addr = (segments[0] >> 4);
    uss_option = (segments[0] >> 2 & 0b11);

    switch (segments[0] & 0b11) {
    case 0:
        switch (uss_option) {
        case 0: uss_period[uss_addr] = 0; uss_oneShot[uss_addr] = false; Serial1.write(0xff); return true;
        case 1: uss_oneShot[uss_addr] = true; Serial1.write(0xff); return true;
        case 2: uss_oneShot[uss_addr] = false; uss_period[uss_addr] = getExtraData(); Serial1.write(0xff); return true;
        }

    case 1: uss_measureUnit[uss_addr] = uss_option; Serial1.write(0xff); return true;
    case 2: uss_delay[uss_addr] = getExtraData(); Serial1.write(0xff); return true;
    case 3:

        Serial1.write(uss_addrs[uss_addr]);

        Serial1.write(uint8_t(uss_delay[uss_addr] >> 8));
        Serial1.write(uint8_t((uss_delay[uss_addr])));

        Serial1.write(uint8_t(uss_period[uss_addr] >> 8));
        Serial1.write(uint8_t((uss_period[uss_addr])));
        return true;

    default:
        if (segments[0] == 255) {
            Serial1.write(uss_addrs[0]);
            Serial1.write(uss_addrs[1]);
            return true;
        }
    }


    return false;
}

// TODO
void sensorController(uint8_t uss_index) {

    uss_measure(uss_addrs[uss_index],
        uss_measureUnit[uss_index] == 0 ? REAL_RANGING_MODE_INCHES : uss_measureUnit[uss_index] == 1 ? REAL_RANGING_MODE_CMS : REAL_RANGING_MODE_USECS,
        &(uss_measure_data[uss_index]), &(uss_autotune_data[uss_index]));

    uss_time[uss_index] = millis();
    oledController();

    delay(uss_delay[uss_index]);

    if (uss_oneShot[uss_index]) {
        uss_oneShot[uss_index] = false;
        uss_period[uss_index] = 0;
    }
}

uint16_t getExtraData() {
    return uint16_t((segments[1] << 8) | segments[2]);
}


void oledController() {

    oled.clear();

    oled.println("Sensor 0: " + String(uss_measure_data[0]));
    oled.println(String(uss_measureUnit[0] == 0 ? "inc" : uss_measureUnit[0] == 1 ? " cm" : " ms") + " min= " + String(uss_autotune_data[0]));
    oled.println("Sensor 1: " + String(uss_measure_data[1]));
    oled.println(String(uss_measureUnit[0] == 0 ? "inc" : uss_measureUnit[0] == 1 ? " cm" : " ms") + " min= " + String(uss_autotune_data[1]));
}