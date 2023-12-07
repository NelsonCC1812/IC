// *=> imports
#include "lora.controller.h"

// *=> control constants
#define USE_SERIAL 1

// *=> main

void setup() {
    if (USE_SERIAL) {
        Serial.begin(9600);
        while (!Serial) {}
    }


}

void loop() {

}