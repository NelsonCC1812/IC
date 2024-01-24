// imports
#include "lora.controller.h"

// *=> declares
#define LORA_ADDR 7

// *=> consts
extern lora_t lora;

// *=> headers
void onReceive(LoraMessage_t msg);

// *=> setup
void setup() {

    Serial.begin(9600);
    while (!Serial) {}

    lora.init(LORA_ADDR, onReceive);
    lora.applyConfig({ 0, 7, 5, 2 }, CM_CONFS_MASK);
    lora.receive();
}

// *=> loop
void loop() {
}


// *=> functions implementations

void onReceive(LoraMessage_t msg) {
    Serial.println("{\"" + String(msg.sender)
        + "\":{\"status\":" + String(msg.payload[0])
        + ", \"isFull\":" + String(msg.payload[1]) "}"
        "}");
}