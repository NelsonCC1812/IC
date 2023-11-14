// *=> imports
#include <Wire.h>
#include <SSD1306AsciiWire.h>

#include "oled_commands.h"
#include "uss_commands.h"

// constants
#define COMMAND_SIZE 4

// *=> vars

String command[COMMAND_SIZE] = { "", "", "", "" };  // almacena el comando
String str;                             // almacena el string que leemos por consola
String cmp = "algo";                    // -
uint8_t idx, idx0, idx1;                // punteros para el buildCommand
uint8_t tmp;                            // int temporal para algunos calculos internos



// *=> main

void setup() {
    Serial.begin(9600);
    while (!Serial) {}

    Serial.println("> Start");
}

void loop() {

    if (getConsoleData() && buildCommand())
        Serial.println(command[0] + command[1] + command[2] + command[3] + "-");

    //if (str.length() > 0)  Serial.println("> " + String(str == cmp));
}

// *=> utilities

bool getConsoleData() {
    str = "";
    if (Serial.available()) {
        str = Serial.readString();
        str.trim();
        return true;
    }

    return false;
}

bool buildCommand() {

    for (idx = 0; idx < COMMAND_SIZE; ++idx) command[idx] = "";
    idx0 = 0, idx = 0;

    while (idx0 < str.length() && idx < COMMAND_SIZE) {
        idx1 = str.indexOf(" ", idx0 + 1);
        command[idx++] = str.substring(idx0, idx1);

        idx0 = idx1;

        while (str.charAt(idx0 + 1) == ' ' && idx0 < str.length()) idx0++;
    }


    return str.indexOf(" ", idx0) == str.lastIndexOf("") ? true : false;
}

// bool buildSegment() {

// }


// console commands
bool c_help() {
    Serial.println("\nCommands:");
    Serial.println("\tus <srf02> {one-shot | on <period_ms> | off}");
    Serial.println("\tus <srf02> unit {inc | cm | ms}");
    Serial.println("\tus <srf02> delay <ms>");
    Serial.println("\tus <srf02> status");
    Serial.println("\tus");
    Serial.println();
    return true;
}



/** // TODO
 *
 *  00: 1 |         | 0: off, 1: one-shot, num: period_ms (si es 1 se le añade un 0 delante o al final)
 *  01: 2 | unit    | 0: inc, 1: cm, -: ms
 *  10: 3 | delay   | num
 *  11: 4 | status  | 0: 0, 1: 1
*/