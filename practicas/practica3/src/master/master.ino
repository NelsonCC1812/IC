// *=> imports
#include "lora.controller.h"

// *=> control constants
#define USE_SERIAL 1

#define ADDR_LOCAL  0xE0
#define ADDR_DEST   0xE0

// *=> var
uint8_t py[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };

extern lora_t lora;

// *=> main

void setup() {
    if (USE_SERIAL) {
        Serial.begin(9600);
        while (!Serial) {}
        Serial.println("Serial started");
    }


    lora.init(ADDR_LOCAL, ADDR_DEST, onReceive);
    Serial.println("LoRa init");

    LoraConfig_t config = { 9,7, 5, 2 };
    lora.applyConfig(config, 0b11111);
    Serial.println("LoRa after config");
    lora.receive();
}

void loop() {
    lora.sendMessage(0, py, 3);
    delay(1000);
    //lora.receive();
}

// *=> function implementations
void onReceive(LoraMessage_t message) {
    Serial.print("Recieve");
    Serial.println(message.payloadLength);
    for (int i = 0; i < message.payloadLength; i++) {
        Serial.print(message.payload[i]);
        Serial.print(" ");
    }
    Serial.print(message.id);
    Serial.println();
}