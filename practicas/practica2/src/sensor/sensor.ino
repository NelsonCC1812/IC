// *=> imports
#include <Wire.h>
#include <SSD1306AsciiWire.h>

#include "oled_commands.h"
#include "uss_commands.h"

// *=> vars
byte segments[2];
uint8_t idx;


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

    receiveSegments() && Serial.println("> " + String(segments[0]) + " " + String(segments[1]));

}


bool receiveSegments() {
    idx = 0;
    if (Serial1.available()) {
        while (Serial1.available() && idx < 2) {
            segments[idx++] = Serial1.read();
        }
        return true;
    }

    return false;
}