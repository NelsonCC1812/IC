// imports
#include "lora.controller.h"

// *=> declares
#define LORA_ADDR 7
#define ALLOWED_ADDRS 3

// *=> consts
extern lora_t lora;
const uint8_t allowed_addrs[3] = {10, 20, 30};

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


void onReceive(LoraMessage_t msg) {

    for(int i = 0; i < ALLOWED_ADDRS; i++){
        if(allowed_addrs[i] == msg.sender){

            Serial.println(
        "{\"addr\":" + String(msg.sender) + ","
        + "\"isFull\":" + String(msg.payload[0])
        + "}");
        
            return;
        }
    }

    
}