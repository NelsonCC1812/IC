// *=> imports
#include <Wire.h>

// constants
#define COMMAND_SIZE 4
#define US_QUANTITY 2


// *=> vars

String command[COMMAND_SIZE] = { "", "", "", "" };  // almacena el comando
String str;                                        // almacena el string que leemos por consola & string de apoyo
uint8_t idx, idx0, idx1;                            // punteros para el buildCommand
uint8_t tmp;                                        // int temporal para algunos calculos internos
uint8_t extra;

String cmp = "algo";                                // -


// function headers
bool getConsoleData();
bool buildCommand();
bool playCommand();
bool buildSegment();
bool sendSegment(String segment, bool haveExtra = false);
bool receiveSegment();



// *=> main

void setup() {
    Serial.begin(9600);
    while (!Serial) {}

    Serial.println("> Start");
}

void loop() {

    if (getConsoleData())if (!buildCommand()) continue;
    buildSegment();

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

// TODO
bool playCommand() {

    // command[0]
    if (command[0] == "help") { c_help(); return true; }
    if (command[0] != "us") { Serial.println("ERROR: El comando debe comenzar por us\n\tusa el comando help para mas informacion"); return false; }

    // command[1]
    if (command[1].length() != 1) {
        Serial.println("ERROR: El segundo parametro indica el sensor (0" + String(US_QUANTITY - 1) +
            ")\t\nusa el comando help para mas informacion");
        return false;
    }
    if (command[1] == "") { c_commandc5(); return true; }

    tmp = command[1].chatAt(0) - '0';
    if (tmp < 0 || tmp > US_QUANTITY) {
        Serial.println("ERROR: El segundo parametro indica el sensor (0" + String(US_QUANTITY - 1) +
            ")\t\nusa el comando help para mas informacion");
        return false;
    }

    str = command[1];

    // command[2]
    if (command[2] == "") { Serial.println("ERROR: El comando es incorrecto\n\tusa el comando help para mas informacion"); return false; }
    if (command[2] == "off") { sendSegment(str + "000"); return true; }
    if (command[2] == "one-shot") { sendSegment(str + "001"); return true; }
    if (command[2] == "on") {
        if (command[1] == "") {
            Serial.println("ERROR: El parametro que acompaña a la opción on, no puede estar vacio\n\tusa el comando help para mas informacion");
            return false;
        }

        tmp = String(command[3]).toInt();
        if (tmp <= 0) {
            Serial.println("ERROR: El paramentro que acompaña a la opción tiene que se un numero mayor a 0");
            return false;
        }

        extra = tmp;
        sendSegment(str + "00", true);
        return true;
    }
    if (command[2] == "unit") {
        if (command[3] == "") {
            Serial.print("ERROR: En el comando unit hay que especificar la unidad de medida\n\tusa el comando help para mas informacion");
            return false;
        }

        if (command[3] == "inc") { sendSegment(str + "010"); return true; }
        if (command[3] == "cm") { sendSegment(str + "011"); return true; }
        if (command[3] == "ms") { sendSegment(str + "01"); return true; }

        Serial.println("ERROR: Las opciones para el comando unit son {inc, cm, ms}\n\tusa el comando help para mas informacion");
        return false;
    }
    if (command[2] == "delay") {
        if (command[3] == "") { Serial.println("ERROR: El comando delay debe ir acompañado de la medida en ms\n\tusa el comando help para mas informacion"); return false; }
        extra = String(command[3]).toInt();
        if (extra <= 0) { Serial.println("ERROR: El delay debe ser un numero positivo"); return false; }

        sendSegment(str + "11", true);
        return true;
    }
    if (command[2] == "status") { sendSegment(str + "11"); return true; }


    Serial.println("ERROR: Haz introducido un comando incorrecto\n\tusa el comando help para mas informacion");
    return false;
}


// TODO
bool sendSegment(String segment, bool haveExtra) {

}


// TODO
String receiveSegment() {

}


// *=> console commands
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