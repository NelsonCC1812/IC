// *=> imports
#include "lora.controller.h"

// *=> control constants
#define USE_SERIAL 1

#define ADDR_LOCAL  0xE0
#define ADDR_DEST   0xE1

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


    lora.init(ADDR_LOCAL, onReceive);
    lora.autoReceive = true;
    lora.canSendConfig = true;
    lora.hasDynamicConfig = true;

    if (lora.applyConfig({ 3,9, 5, 2 }, 0b10001111)) Serial.println("Changed config");
    lora.receive();

    Serial.println("LoRa init");
}

void loop() {
    Serial.println("mandar mensaje en main");
    Serial.println(String(lora.msgCount) + " " + String(lora.sendMessage(ADDR_DEST, OPBIT_ACK_WAITING, py, 3, false)));
    delay(5000);
}

// *=> function implementations
void onReceive(LoraMessage_t message) {
    Serial.println();
    Serial.println("Received =================================================");
    Serial.println("SPF " + String(lora.config.spreadingFactor));
    Serial.println("BW " + String(lora.config.bandwidth_index));

    for (int i = 0; i < 5; i++) {
        Serial.print(message.payload[i]);
        Serial.print(" ");
    }

    Serial.print(" ID: " + String(message.id));
    Serial.println(message.id);
    Serial.println("==========================================================");
    Serial.println();
}