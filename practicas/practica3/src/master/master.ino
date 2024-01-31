// *=> imports
#include "lora.controller.h"

// *=> control constants
#define USE_SERIAL 1

#define ADDR_LOCAL  0xE0
#define ADDR_DEST   0xE1

#define MESSAGE_PERIOD 10000

// *=> var
uint8_t py[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };

extern lora_t lora;


uint32_t time = MESSAGE_PERIOD;

// *=> main

void setup() {
    if (USE_SERIAL) {
        Serial.begin(9600);
        while (!Serial) {}
        Serial.println("Serial started");
    }


    lora.init(ADDR_LOCAL, onReceive);
    lora.isMaster = true;

    lora.receive(false);

    Serial.println("LoRa init");
}

void loop() {
    if ((millis() - time) > MESSAGE_PERIOD) {
        Serial.println("Mensaje en main " + String(lora.msgCount) + " " + String(lora.sendMessage(ADDR_DEST, OPBIT_ACK_WAITING, py, 3)));
        time = millis();
    }
    lora.control();
}

// *=> function implementations
void onReceive(LoraMessage_t message) {
    Serial.println();
    Serial.println("Received =================================================");
    Serial.println("SPF " + String(lora.config.spreadingFactor));
    Serial.println("BW " + String(lora.config.bandwidth_index));
    Serial.println("PWR: " + String(lora.config.txPower));

    for (int i = 0; i < 5; i++) {
        Serial.print(message.payload[i]);
        Serial.print(" ");
    }

    Serial.println(" ID: " + String(message.id));
    Serial.println("==========================================================");
    Serial.println();
}