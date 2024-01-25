#include <arduino.h>
#include <Wire.h>
#include <uss_commands.h>

#define USS_COMMAND 0x00

// *=> headers
// void uss_writeCommand(byte addr, byte command);
// byte uss_readRegister(byte addr, byte the_register);
// void uss_measure(uint8_t addr, uint16_t* measure, uint16_t* autotune);

// *=> implementation

void uss_writeCommand(byte addr, byte command) {
    Wire.beginTransmission(addr);
    Wire.write(USS_COMMAND);
    Wire.write(command);
    Wire.endTransmission();
}

byte uss_readRegister(byte addr, byte the_register) {
    Wire.beginTransmission(addr);
    Wire.write(the_register);
    Wire.endTransmission();


    Wire.requestFrom(addr, byte(1));
    while (!Wire.available()) { Serial.println("bucle infinito"); delay(1000); }
    return Wire.read();
}

void uss_measure(uint8_t addr, byte mode, uint16_t* measure, uint16_t* autotune) {
    uss_writeCommand(addr, mode);
    delay(100);

    *measure = uint16_t((uss_readRegister(addr, RANGE_HIGH_BYTE) << 8) | uss_readRegister(addr, RANGE_LOW_BYTE));
    *autotune = uint16_t((uss_readRegister(addr, AUTOTUNE_MINIMUM_HIGH_BYTE) << 8) | uss_readRegister(addr, AUTOTUNE_MINIMUM_LOW_BYTE));
}