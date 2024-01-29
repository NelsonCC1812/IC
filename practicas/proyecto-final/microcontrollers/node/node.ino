#include <Wire.h>

#include "lora.controller.h"
#include "uss_commands.h"

// *=> declares
#define LORA_ADDR 10
#define GATEWAY_ADDR 7
#define USS_ADDR byte(0xE0 >> 1) 
#define GAP 20
#define WIRE_INIT_DELAY 100
#define BAUDRATE 9600
#define PERIOD 10000



// *=> consts
extern lora_t lora;
const LoraConfig_t lora_config = { 0, 7, 5, 2 };

// *=> vars
uint8_t msg[1];
uint16_t autotune;
uint16_t measure;

// *=> headers
void onReceive(LoraMessage_t msg);

// *=> setup
void setup() {

    Serial.begin(BAUDRATE);
    while (!Serial) {}

    Wire.begin();
    delay(WIRE_INIT_DELAY);


    lora.init(LORA_ADDR, onReceive);
    lora.applyConfig(lora_config, CM_CONFS_MASK);
}

// *=> loop
void loop() {
    uss_measure(USS_ADDR, REAL_RANGING_MODE_CMS, &measure, &autotune);
    msg[0] = (measure <= GAP);
    Serial.println(msg[0]);
    lora.sendMessage(GATEWAY_ADDR, OPCODE_DATA, msg, 1, false);
    delay(PERIOD);
}

void onReceive(LoraMessage_t msg) {
    return;
}