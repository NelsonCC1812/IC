#include <Wire.h>
#include <SSD1306AsciiWire.h>

#include "oled_commands.h"
#include "uss_commands.h"

extern SSD1306AsciiWire oled;

void setup() {
    Wire.begin();
    oled.init(0);
}

uint8_t idx = 0;
char bff[100];

void loop() {
    snprintf(bff, 100, "%d", idx++);
    oled.print(bff);
    delay(1000);
}

/**
 * => Leer
 * if( Serial1.available() > 0){
 * data = Serial.read()
 * }
 *
 * => Escribir
 * Serial1.write(cosas)
*/