// *=> imports
#include <arduino.h>

#include <SPI.h>             
#include <LoRa.h>

#include "lora.controller.h"


// declare

// srn & rssi
float SNR_REAL_MIN_DB(uint8_t spf) { return 10 - 2.5 * spf; }
float SNR_MIN(uint8_t spf) { return (10 - 2.5 * spf) * SNR_MIN_GAP; }

#define RSSI_MIN_REAL -120
#define RSSI_MIN()(RSSI_MIN_REAL*(1-RSSI_MIN_GAP))
#define RSSI_MAX -50

// *=> config params
#define BW_MASK 0b1111
#define BW_BITS 4
#define BW_MIN 0
#define BW_MAX 9

#define SPF_MASK 0b111
#define SPF_BITS 3
#define SPF_MIN 7
#define SPF_MAX 12

#define CR_MASK 0b11
#define CR_BITS 2
#define CR_MIN 5
#define CR_MAX 8

#define PWR_MASK 0b11111
#define PWR_BITS 5
#define PWR_MIN 2
#define PWR_MAX 20

#define CONFIG_SIZE 2

// *=> system constants

#define BW_SIZE 10


// *=> consts
void (*onReceive_call)(LoraMessage_t);
const double bandwidth_kHz[BW_SIZE] = { 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };


// *=> vars
volatile bool transmitting = false;

uint8_t receivedBytes;
uint8_t currentNode;

uint16_t lora_tmp16;
uint8_t config_payload[2];

volatile uint32_t lastSendTime_ms = 0;
volatile uint32_t txInterval_ms = TX_LAPSE_MS;
uint32_t tx_begin_ms;
uint32_t TxTime_ms;
uint32_t lapse_ms;
float duty_cycle;

uint32_t connection_try_time_ms;

uint8_t payload[PAYLOAD_SIZE];

LoraConfig_t tmpConfig;

lora_t lora;

uint8_t hasToSendAck = 0;

bool sendingMessage = false;
bool waitsForAck = 0;
volatile uint8_t hasToSendConfig = 0;
volatile bool hasToChangeConfig = 0;

volatile bool receivedMessage = false;
uint32_t lastConnection = 0;
bool revertedConfig = false;


// *=> headers (private)
bool trySendMessage(uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength);

void onReceive(int packetSize);
void TxFinished();

LoraConfig_t extractConfig(byte payload[PAYLOAD_SIZE]);
bool isValidConfig(LoraConfig_t config);
LoraConfig_t refitConfig();
void lifePulse();

uint8_t calcSpfThroughtBW(uint8_t bw_index);

// *=> implementations

// public

bool _init(uint8_t localAddr, void (*onReceive_func) (LoraMessage_t msg)) {
    lora.localAddr = localAddr;
    onReceive_call = onReceive_func;

    if (!LoRa.begin(FREC_BAND)) return false;

    LoRa.setSyncWord(syncWord);
    LoRa.setPreambleLength(8);
    LoRa.onReceive(onReceive);
    LoRa.onTxDone(TxFinished);

    lora.resetConfig();

    LoRa.receive();

    return true;
}


bool _sendMessage(uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength) {

    Serial.println("SendMessage");
    waitsForAck = false;

    if (destAddr != BROADCAST_ADDR && opCode & OPBIT_ACK_WAITING) waitsForAck = true;

    lora.node.ack = false;

    if (!waitsForAck) {
        lora.node.waitingAck = false;
        trySendMessage(destAddr, opCode, payload, payloadLength);
        lora.msgCount++;
        return false;
    }

    lora.node.waitingAck = true;

    if (!sendingMessage) sendingMessage = true;

    for (int i = 0; i < CONNECTION_TRY_TIMES; i++) {

        trySendMessage(destAddr, opCode | OPBIT_ACK_WAITING, payload, payloadLength);

        while (!lora.receive(false)) {}

        connection_try_time_ms = millis();

        while ((millis() - connection_try_time_ms) < CONNECTION_TRY_TIMEOUT_MS) {
            if (lora.node.ack) {
                lora.msgCount++;
                sendingMessage = false;
                Serial.println("Received ACK ");
                return true;
            }
        }
    }


    lora.receive(false);
    sendingMessage = false;
    Serial.println("No received ACK");
    return false;
}

bool _receive(bool force) {

    if (transmitting) return false;

    if (force) {
        LoRa.receive();
        lora.isReceiving = true;

        return true;
    }


    if (lora.isReceiving) return true;

    LoRa.receive();
    lora.isReceiving = true;

    return true;
}

void _resetConfig() {
    lora.applyConfig(BASE_CONFIG);
}

bool _applyConfig(LoraConfig_t config) {

    Serial.println("Trying to apply configuration");

    if (!isValidConfig(config)) return false;
    Serial.println("configuracion valida");


    while (transmitting) {}

    lora.lastConfig = lora.config;
    lora.config = config;

    LoRa.setSignalBandwidth(long(bandwidth_kHz[config.bandwidth_index]));
    LoRa.setSpreadingFactor(config.spreadingFactor);
    LoRa.setCodingRate4(config.codingRate);
    LoRa.setTxPower(config.txPower, PA_OUTPUT_PA_BOOST_PIN);


    LoRa.sleep();
    delay(100);
    while (!lora.receive(true)) {}
    delay(100);

    Serial.println("Applied Config ================");
    Serial.println("BW: " + String(config.bandwidth_index));
    Serial.println("SPF: " + String(config.spreadingFactor));
    Serial.println("CR: " + String(config.codingRate));
    Serial.println("PWR: " + String(config.txPower));
    Serial.println("================================");

    return true;
}


bool _sendConfig(uint8_t destAddr, LoraConfig_t config) {
    lora_tmp16 = 0;

    lora_tmp16 |= (config.bandwidth_index);

    lora_tmp16 <<= SPF_BITS;
    lora_tmp16 |= (config.spreadingFactor - SPF_MIN);

    lora_tmp16 <<= CR_BITS;
    lora_tmp16 |= (config.codingRate - CR_MIN);

    lora_tmp16 <<= PWR_BITS;
    lora_tmp16 |= (config.txPower - PWR_MIN);

    config_payload[0] = uint8_t(lora_tmp16 >> 8);
    config_payload[1] = uint8_t(lora_tmp16);


    return lora.sendMessage(destAddr, OPBIT_CONFIG, config_payload, CONFIG_SIZE);
}

LoraConfig_t extractConfig(byte payload[PAYLOAD_SIZE]) {
    LoraConfig_t config;

    lora_tmp16 = (uint16_t)((payload[1]) | uint16_t(payload[0] << 8));

    config.txPower = (lora_tmp16 & PWR_MASK) + PWR_MIN;
    lora_tmp16 >>= PWR_BITS;

    config.codingRate = (lora_tmp16 & CR_MASK) + CR_MIN;
    lora_tmp16 >>= CR_BITS;

    config.spreadingFactor = (lora_tmp16 & SPF_MASK) + SPF_MIN;
    lora_tmp16 >>= SPF_BITS;

    config.bandwidth_index = lora_tmp16 & BW_MASK;

    return config;
}

void onReceive(int packetSize) {

    Serial.println("onReceive");

    if (packetSize == 0) return;

    currentNode = LoRa.read();

    lora.node.msg.sender = currentNode;
    lora.node.msg.rcpt = LoRa.read();

    lora.node.msg.id = ((uint16_t)(LoRa.read() << 8) | (uint16_t)LoRa.read());
    lora.node.msg.opCode = LoRa.read();

    lora.node.msg.payloadLength = LoRa.read();

    if (lora.node.msg.payloadLength > PAYLOAD_SIZE) { lora.node.msg.err = ERR_PAYLOAD_EXCEDES_BUFFER; return; }

    receivedBytes = 0;
    while (LoRa.available() && (receivedBytes < lora.node.msg.payloadLength)) lora.node.msg.payload[receivedBytes++] = LoRa.read();

    if (lora.node.msg.payloadLength != receivedBytes) { lora.node.msg.err = ERR_PAYLOAD_NOT_COINCIDES; return; }
    if ((lora.node.msg.rcpt & lora.localAddr) != lora.localAddr) { lora.node.msg.err = ERR_TARGET_ERROR; return; }
    if (LoRa.available() && (LoRa.read() != END_SEGMENT)) { lora.node.msg.endReceived = false; lora.node.msg.err = ERR_END_NOT_RECEIVED; return; }

    lora.node.msg.err = NO_ERROR;

    lora.node.msg.rssi = LoRa.packetRssi();
    lora.node.msg.snr = LoRa.packetSnr();

    if (lora.node.msg.snr <= SNR_REAL_MIN_DB(lora.config.spreadingFactor))
        lora.node.msg.err = ERR_NOISE_EXCEDES_SIGNAL;

    if (lora.node.msg.opCode == OPCODE_ACK && lora.node.waitingAck) {
        lora.node.waitingAck = false;
        lora.node.ack = true;
    }

    if (lora.node.msg.opCode & OPBIT_ACK_WAITING) hasToSendAck = currentNode;

    if (lora.isMaster) {
        tmpConfig = refitConfig();
        if (tmpConfig.txPower) hasToSendConfig = currentNode;
    }

    if (lora.node.msg.opCode & OPBIT_CONFIG) {
        hasToChangeConfig = true;
    }


    receivedMessage = true;

    if (onReceive_call) onReceive_call(lora.node.msg);

}

void _control() {

    if (receivedMessage) {
        lastConnection = millis();
        receivedMessage = false;
        revertedConfig = false;
    }

    if (hasToSendAck) {
        lora.sendMessage(hasToSendAck, OPCODE_ACK, payload, 2);
        hasToSendAck = false;
    }

    if (hasToChangeConfig && !lora.isMaster) {
        lora.applyConfig(extractConfig(lora.node.msg.payload));
        hasToChangeConfig = false;
        revertedConfig = false;
    }


    if (hasToSendConfig && lora.isMaster) {
        lora.sendConfig(hasToSendConfig, tmpConfig);
        lora.applyConfig(tmpConfig);

        if (!lora.sendMessage(hasToSendConfig, OPCODE_PING | OPBIT_ACK_WAITING, NULL, 0)) {
            Serial.println("Resetea a config anterior tras mandar");
            lora.applyConfig(lora.lastConfig);
            lora.lastConfig = BASE_CONFIG;
            revertedConfig = true;
        }

        hasToSendConfig = 0;
    }


    if (lastConnection && (millis() - lastConnection) > (REVERT_BASE_CONFIG_TIMEOUT)) {
        Serial.println("MILLIS: " + String(millis()) + " " + String(lastConnection));
        Serial.println("Resetea a config base timeout *5");
        lora.resetConfig();
        lastConnection = 0;
        revertedConfig = false;
    }
    else if (lastConnection && !revertedConfig && (millis() - lastConnection) > REVERT_CONFIG_TIMEOUT) {
        Serial.println("MILLIS: " + String(millis()) + " " + String(lastConnection));
        Serial.println("Resetea a config anterior timeout");
        lora.applyConfig(lora.lastConfig);
        lora.lastConfig = BASE_CONFIG;
        revertedConfig = true;
    }
}


void TxFinished() {
    if (!transmitting) return;

    transmitting = false;

    TxTime_ms = millis() - tx_begin_ms;
    lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms;

    duty_cycle = (100.0f * TxTime_ms) / lapse_ms;

    Serial.println("duty cycle: " + String(duty_cycle) + "%");


    if (duty_cycle > DUTY_CYCLE_MAX) txInterval_ms = TxTime_ms * DUTY_CYCLE_MAX * 100;

    lora.receive(false);
}

bool trySendMessage(uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength) {

    while (transmitting || ((millis() - lastSendTime_ms) < txInterval_ms)) {}

    while (!LoRa.beginPacket()) { delay(10); }

    LoRa.write(lora.localAddr);
    LoRa.write(destAddr);

    LoRa.write((uint8_t)(lora.msgCount >> 8));
    LoRa.write((uint8_t)(lora.msgCount & 0xFF));

    LoRa.write(opCode);

    LoRa.write(payloadLength);
    LoRa.write(payload, (size_t)payloadLength);

    LoRa.write(END_SEGMENT);

    LoRa.endPacket(true);

    tx_begin_ms = millis();

    transmitting = true;
    lora.isReceiving = false;

    return true;
}

bool isValidConfig(LoraConfig_t config) {
    if (config.bandwidth_index < BW_MIN || config.bandwidth_index > BW_MAX) return false;
    if (config.spreadingFactor < SPF_MIN || config.spreadingFactor > SPF_MAX) return false;
    if (config.codingRate < CR_MIN || config.codingRate > CR_MAX) return false;
    if (config.txPower < PWR_MIN || config.txPower > PWR_MAX) return false;

    return true;
}

LoraConfig_t refitConfig() {
    bool hasBeingModified = false;

    LoraMessage_t msg = lora.node.msg;
    LoraConfig_t config;

    config.bandwidth_index = lora.config.bandwidth_index;
    config.codingRate = lora.config.codingRate;
    config.spreadingFactor = lora.config.spreadingFactor;
    config.txPower = lora.config.txPower;


    if (msg.rssi <= RSSI_MIN() && config.txPower < PWR_MAX) {
        hasBeingModified = true;
        config.txPower++;
    }
    if (msg.rssi > RSSI_MAX && config.txPower > PWR_MIN) {
        hasBeingModified = true;
        config.txPower--;
    }



    if (msg.snr < SNR_MIN(lora.config.spreadingFactor) && config.bandwidth_index > BW_MIN) {
        hasBeingModified = true;

        config.bandwidth_index--;
        config.spreadingFactor = calcSpfThroughtBW(config.bandwidth_index);
    }

    Serial.println("SNR :" + String(msg.snr) + " | RSSI: " + String(msg.rssi));
    if (msg.snr > SNR_MAX && config.bandwidth_index < BW_MAX) {
        hasBeingModified = true;

        config.bandwidth_index++;
        config.spreadingFactor = calcSpfThroughtBW(config.bandwidth_index);
    }


    return  hasBeingModified ? config : (LoraConfig_t) { 0, 0, 0, 0 };
}


uint8_t calcSpfThroughtBW(uint8_t bw_index) {
    return  bw_index * (SPF_MIN - SPF_MAX) / (BW_MAX - BW_MIN) + SPF_MAX;
}