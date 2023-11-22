// *=> imports
#include <Wire.h>

// constants
#define COMMAND_SIZE 4
#define US_QUANTITY 2
#define MAX_US_ADDR 15
#define MIN_US_ADDR 0
#define MAX_SERIAL_TIME 100


// *=> vars

String command[COMMAND_SIZE] = { "", "", "", "" };  // almacena el comando
String str;                                        // almacena el string que leemos por consola & string de apoyo
uint8_t idx, idx0, idx1;                            // punteros para el buildCommand
uint16_t tmp;                                        // int temporal para algunos calculos internos
uint16_t extra;
byte segment;
byte segments[2];
uint8_t code = 255; // on: 0, us: 5, status: 4
uint32_t time;

// function headers
bool getConsoleData();
bool buildCommand();
bool playCommand();
bool buildSegment();
bool sendSegment(byte segment, bool haveExtra = false);
bool receiveSegments();
bool playSegment();

bool c_help();


// *=> main

void setup() {
    Serial.begin(9600);
    while (!Serial) {}

    Serial1.begin(9600);
    while (!Serial1) {}

    Serial.println("> Start");

}

void loop() {
    getConsoleData() && buildCommand() && playCommand();
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

        while (str.charAt(idx0) == ' ' && idx0 < str.length()) idx0++;
    }

    return str.indexOf(" ", idx0) == str.lastIndexOf("") ? true : false;
}



bool playCommand() {

    segment = 0;
    code = 255;

    // command[0]
    if (command[0] == "help") { c_help(); return true; }
    if (command[0] != "us") { Serial.println("ERROR: El comando debe comenzar por us\n\tusa el comando help para mas informacion"); return false; }

    // command[1]
    if (command[1] == "") { code = 5; sendSegment(0xFF); return true; }

    tmp = command[1].toInt();
    if (tmp < MIN_US_ADDR || tmp > MAX_US_ADDR) {
        Serial.println("ERROR: El segundo parametro indica el sensor (0-" + String(US_QUANTITY - 1) +
            ")\t\nusa el comando help para mas informacion");
        return false;
    }

    segment |= (tmp << 4);

    // command[2]
    if (command[2] == "") { Serial.println("ERROR: El comando es incorrecto\n\tusa el comando help para mas informacion"); return false; }
    if (command[2] == "off") { sendSegment(segment); return true; }
    if (command[2] == "one-shot") { sendSegment(segment | 0b100); return true; }
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
        sendSegment(segment | 0b1000, true);
        return true;
    }
    if (command[2] == "unit") {
        if (command[3] == "") {
            Serial.print("ERROR: En el comando unit hay que especificar la unidad de medida\n\tusa el comando help para mas informacion");
            return false;
        }

        if (command[3] == "inc") { sendSegment(segment | 0b1); return true; }
        if (command[3] == "cm") { sendSegment(segment | 0b101); return true; }
        if (command[3] == "ms") { sendSegment(segment | 0b1001); return true; }

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
    if (command[2] == "status") { code = 4; sendSegment(segment | 0b11); return true; }


    Serial.println("ERROR: Haz introducido un comando incorrecto\n\tusa el comando help para mas informacion");
    return false;
}


bool sendSegment(byte segment, bool haveExtra) {
    Serial1.write(segment);
    haveExtra&& Serial1.write(extra);

    if ((code != 4 || code != 5)) {
        if (receiveSegments() == 0xFF) {
            Serial.println("Comando ejecutado correctamente");
            return true;
        }

        Serial.println("La direccion usada para el sensor no es valida");
    }

    receiveSegments();
    playSegment();
    return true;
}


bool receiveSegments() {
    for (idx = 0; idx < 2; idx++) segments[idx] = 0;
    idx = 0;
    if (Serial1.available()) {
        time = millis();
        if (Serial1.available()) {
            while ((millis() - time) < MAX_SERIAL_TIME) {
                if (Serial1.available())
                    segments[idx++] = Serial1.read();
                if (idx >= 2) return true;
            }
            return true;
        }
    }

    return false;
}


bool playSegment() {

    if (code == 4) {
        Serial.println("\nAddress: " + String(segments[0] & 0b11110)
            + "\nMin-delay: " + String(int(segments[1]))
            + "\nPeriodic-on: " + (segments[0] & 0b1 ? "yes" : "no"));
        return true;
    }

    if (code == 5) {
        Serial.println("\nSensor 0: " + String(segments[0] & 0b1111)
            + "\nSensor 1: " + String(segments[0] & ~0b1111));
        return true;
    }

    return false;
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
 * invertido codigo operacion: 2 | sensor 0-16 | opcion: 1-2
 *
 *  00: 1 |         | 00: off, 01: one-shot, 11: period_ms (si es 1 se le añade un 0 delante o al final)
 *  01: 2 | unit    | 00: inc, 01: cm, 11: ms
 *  10: 3 | delay   | num
 *  11: 4 | status  | 0: 0, 1: 1
*/