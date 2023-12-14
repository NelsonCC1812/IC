// *=> imports
#include "lora.controller.h"

// *=> control constants
#define USE_SERIAL 1

#define ADDR_LOCAL  0xE0
#define ADDR_DEST   0xE1

// *=> var
uint8_t py[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
#define IS_SENDER 1

extern lora_t lora;

// *=> main

void setup() {
    if (USE_SERIAL) {
        Serial.begin(9600);
        while (!Serial) {}
        Serial.println("Serial started");
    }


    lora.init(ADDR_LOCAL, onReceive);
    Serial.println("LoRa init");

    LoraConfig_t config = { 0,7, 5, 2 };
    lora.applyConfig(config, 0b1111);
    Serial.println("LoRa after config");
    if (!IS_SENDER) lora.receive();
}

void loop() {
    if (IS_SENDER) Serial.println(String(lora.msgCount) + " " + String(lora.sendMessage(ADDR_DEST, 0, py, 3, false)));
    delay(5000);
}

// *=> function implementations
void onReceive(LoraMessage_t message) {

    for (int i = 0; i < 5; i++) {
        Serial.print(message.payload[i]);
        Serial.print(" ");
    }
    Serial.print(message.id);
    Serial.println();
}