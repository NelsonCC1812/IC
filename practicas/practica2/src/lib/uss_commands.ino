#include <Wire.h>

#define USS_COMMAND 0x00

void uss_writeCommand(byte addr, byte command) {
    Wire.beginTransmission(addr);
    Wire.write(USS_COMMAND);
    Wire.write(command);
    Wire.endTransmission();
}

byte uss_readRegister(byte addr, byte the_register) {
    Wire.beginTransmission(address);
    Wire.write(the_register);
    Wire.endTransmission();

    Wire.requestFrom(byte addr, byte(1));
    while (!Wire.available()) {}
    return Wire.read();
}