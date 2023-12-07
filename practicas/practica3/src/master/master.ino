// *=> imports
#include "lora.controller.h"

// *=> control constants
#define USE_SERIAL 1

#define ADDR_LOCAL  0xE0
#define ADDR_DEST   0xE2

// *=> var
uint8_t py[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };

extern lora_t lora;

// *=> main

void setup() {
    if (USE_SERIAL) {
        Serial.begin(9600);
        while (!Serial) {}
    }

    lora.init(ADDR_LOCAL, ADDR_DEST, onReceive);
}

void loop() {
    lora.sendMessage(0, py, 3);
}

// *=> function implementations
void onReceive(LoraMessage_t message) {
    for (int i = 0; i < message.payloadLength; i++) {
        Serial.print(message.payload[i]);
        Serial.print(" ");
    }
    Serial.print(message.id);
    Serial.println();
}