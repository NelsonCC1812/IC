// *=> imports
#include <Wire.h>
#include <SSD1306AsciiWire.h>

#include "oled_commands.h"
#include "uss_commands.h"

// *=> consts

#define MAX_SERIAL_TIME 100

// *=> vars
byte segments[2];
uint8_t idx;
uint32_t time;


// *=> functions headers

bool receiveSegments();

// *=> main

void setup() {
    Serial.begin(9600);
    while (!Serial) {}

    Serial1.begin(9600);
    while (!Serial1) {}
}

void loop() {

    receiveSegments() && Serial.println("> " + String(segments[0], BIN) + " " + String(segments[1]));

}


bool receiveSegments() {
    for (idx = 0; idx < 2; idx++) segments[idx] = 0;
    idx = 0;
    if (Serial1.available()) {
        time = millis();
        if (Serial1.available()) {
            while ((millis() - time) < MAX_SERIAL_TIME) {
                if (Serial1.available())
                    segments[idx++] = Serial1.read();
                if (idx >= 2) return true;
            }
            return true;
        }
    }

    return false;
}