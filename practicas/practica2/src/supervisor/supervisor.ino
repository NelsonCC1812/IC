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
byte segment;
bool waitingResponse = false;


// function headers
bool getConsoleData();
bool buildCommand();
bool playCommand();
bool buildSegment();
bool sendSegment(String segment, bool haveExtra = false);
String receiveSegment();
bool playSegment(String segment);

bool c_help();
bool c_command5();


// *=> main

void setup() {
    Serial.begin(9600);
    while (!Serial) {}

    Serial.println("> Start");

    Serial.println(String("00100", BIN));
}

void loop() {

    // if (getConsoleData())if (!buildCommand()) continue;
    // buildSegment();
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

    segment = x{ 0 };
    waitingResponse = false;

    // command[0]
    if (command[0] == "help") { c_help(); return true; }
    if (command[0] != "us") { Serial.println("ERROR: El comando debe comenzar por us\n\tusa el comando help para mas informacion"); return false; }

    // command[1]
    if (command[1].length() != 1) {
        Serial.println("ERROR: El segundo parametro indica el sensor (0" + String(US_QUANTITY - 1) +
            ")\t\nusa el comando help para mas informacion");
        return false;
    }
    if (command[1] == "") { c_command5(); return true; }

    tmp = command[1].charAt(0) - '0';
    if (tmp < 0 || tmp > US_QUANTITY) {
        Serial.println("ERROR: El segundo parametro indica el sensor (0" + String(US_QUANTITY - 1) +
            ")\t\nusa el comando help para mas informacion");
        return false;
    }

    tmp == 1 && segment |= 0b100;

    // command[2]
    if (command[2] == "") { Serial.println("ERROR: El comando es incorrecto\n\tusa el comando help para mas informacion"); return false; }
    if (command[2] == "off") { sendSegment(segment); return true; }
    if (command[2] == "one-shot") { sendSegment(segment | 0b1000); return true; }
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
        sendSegment(segment | 0b11000, true);
        return true;
    }
    if (command[2] == "unit") {
        if (command[3] == "") {
            Serial.print("ERROR: En el comando unit hay que especificar la unidad de medida\n\tusa el comando help para mas informacion");
            return false;
        }

        if (command[3] == "inc") { sendSegment(segment | b1); return true; }
        if (command[3] == "cm") { sendSegment(segment | 0b01001); return true; }
        if (command[3] == "ms") { sendSegment(segment | 0b11001); return true; }

        Serial.println("ERROR: Las opciones para el comando unit son {inc, cm, ms}\n\tusa el comando help para mas informacion");
        return false;
    }
    if (command[2] == "delay") {
        if (command[3] == "") { Serial.println("ERROR: El comando delay debe ir acompañado de la medida en ms\n\tusa el comando help para mas informacion"); return false; }
        extra = String(command[3]).toInt();
        if (extra <= 0) { Serial.println("ERROR: El delay debe ser un numero positivo"); return false; }

        sendSegment(segment | 0b10, true);
        return true;
    }
    if (command[2] == "status") { waitingResponse = true; sendSegment(segment | 0b11); return true; }


    Serial.println("ERROR: Haz introducido un comando incorrecto\n\tusa el comando help para mas informacion");
    return false;
}


// TODO
bool sendSegment(byte segment, bool haveExtra) {
    Serial1.write(segment);
    haveExtra&& Serial1.write(extra);

    if (!waitingResponse && receiveSegment() == 0b10100101) {
        Serial.println("Comando ejecutado correctamente");
        return true;
    }

    playSegment(receiveSegment());
    return true;
}

String receiveSegment() {
    str = ""
        while (Serial1.available) {
            str += Serial.readString() + "";
        }
    str = str.trim();
    return str;
}

// TODO
bool playSegment(String segment) {

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

bool c_command5() {
    return false;
}



/** // TODO
 *
 * invertido codigo operacion: 2 | sensor 1 | opcion: 1-2
 *
 *  00: 1 |         | 00: off, 01: one-shot, 11: period_ms (si es 1 se le añade un 0 delante o al final)
 *  01: 2 | unit    | 00: inc, 01: cm, 11: ms
 *  10: 3 | delay   | num
 *  11: 4 | status  | 0: 0, 1: 1
*/