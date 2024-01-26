#include <Wire.h>

#define USS_ORIGINAL_ADDR ((0xE2)>>1)
#define USS_FINAL_ADDR 0xE0
#define USS_COMMAND 0x00
#define USS_DELAY 100

void setup() {

    Serial.begin(9600);

    while (!Serial)
        Serial.println("Starting");

    Wire.begin();
    delay(USS_DELAY);

    writeCommand(0xA0);
    writeCommand(0xAA);
    writeCommand(0xA5);
    writeCommand(USS_FINAL_ADDR);

    Serial.println("Finished");
}

void loop() {

}

void writeCommand(byte command) {
    Wire.beginTransmission(USS_ORIGINAL_ADDR);
    Wire.write(USS_COMMAND);
    Wire.write(command);
    Wire.endTransmission();
}