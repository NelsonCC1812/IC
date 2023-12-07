// *=> imports
#include <arduino.h>
#include <SPI.h>             
#include <LoRa.h>

#include "lora.controller.h"


// declare
#define CONF_MODE_MASK  0b1
#define BW_MASK         0b10
#define SPF_MASK        0b100
#define CR_MASK         0b1000
#define PWR_MASK        0b10000


// *=> consts
const double bandwidth_kHz[10] = { 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };
void (*onReceive_call)(LoraMessage_t) = NULL;

// *=> vars

volatile bool txDoneFlag = true;
volatile bool transmitting = false;

uint8_t receivedBytes;

uint8_t lora_tmp;

uint16_t msgCount = 0;

lora_t lora;


// *=> lora headers
bool _init(uint8_t localAddr, uint8_t destAddr, void  (*onReceive_func) (LoraMessage_t msg));
void _applyConfig(LoraConfig_t config, byte configMask);
void _sendMessage(uint8_t opCode, uint8_t* payload, uint8_t payloadLength, uint16_t msgCount);
LoraConfig_t _extractConfig(byte configMask);
void onReceive(LoraMessage_t message);
void lora_receive();


void TxFinished();
void _onReceive(int packetSize);

// *=> implementations

bool _init(uint8_t localAddr, uint8_t destAddr, void (*onReceive_func) (LoraMessage_t msg)) {

    Serial.println("init");

    lora.localAddr = localAddr;
    lora.destAddr = destAddr;

    onReceive_call = onReceive_func;

    if (!LoRa.begin(FREC_BAND)) return false;

    LoRa.setSyncWord(syncWord);
    LoRa.setPreambleLength(8);
    LoRa.onReceive(_onReceive);
    LoRa.onTxDone(TxFinished);
    LoRa.receive();

    return true;
}

LoraConfig_t _extractConfig(byte configMask) {
    LoraConfig_t config;
    if (!(configMask & CONF_MODE_MASK)) return config;

    lora_tmp = 0;

    if (configMask & BW_MASK)   lora_tmp++;
    if (configMask & SPF_MASK)  lora_tmp++;
    if (configMask & CR_MASK)   lora_tmp++;
    if (configMask & PWR_MASK)  lora_tmp++;

    if (lora.msg.payloadLength > lora_tmp || lora.msg.payloadLength == 0) return config;

    lora_tmp = 0;

    if (configMask & BW_MASK)   config.bandwidth_index = lora.msg.payload[lora_tmp++];
    if (configMask & SPF_MASK)  config.spreadingFactor = lora.msg.payload[lora_tmp++];
    if (configMask & CR_MASK)   config.codingRate = lora.msg.payload[lora_tmp++];
    if (configMask & PWR_MASK)  config.txPower = lora.msg.payload[lora_tmp++];

    return config;
}

void _applyConfig(LoraConfig_t config, byte configMask) {
    if (!(configMask & CONF_MODE_MASK)) return;

    if (configMask & BW_MASK)   LoRa.setSignalBandwidth(long(bandwidth_kHz[config.bandwidth_index]));
    if (configMask & SPF_MASK)  LoRa.setSpreadingFactor(config.spreadingFactor);
    if (configMask & CR_MASK)   LoRa.setCodingRate4(config.codingRate);
    if (configMask & PWR_MASK)  LoRa.setTxPower(config.txPower, PA_OUTPUT_PA_BOOST_PIN);
}

void _sendMessage(uint8_t opCode, uint8_t* payload, uint8_t payloadLength) {

    transmitting = true;
    txDoneFlag = false;

    while (!LoRa.beginPacket()) delay(10);

    LoRa.write(lora.destAddr);
    LoRa.write(lora.localAddr);

    LoRa.write((uint8_t)(msgCount >> 8));
    LoRa.write((uint8_t)(msgCount & 0xFF));

    msgCount++;

    LoRa.write(opCode);

    LoRa.write(payloadLength);
    LoRa.write(payload, (size_t)payloadLength);

    LoRa.write(END_SEGMENT);

    LoRa.endPacket(true);
}


void _onReceive(int packetSize) {
    if (transmitting && !txDoneFlag) txDoneFlag = true;
    if (packetSize == 0) return;

    lora.msg.rcpt = LoRa.read();
    lora.msg.sender = LoRa.read();

    lora.msg.id = ((uint16_t)(LoRa.read() << 8) | (uint16_t)LoRa.read());
    lora.msg.opCode = LoRa.read();

    lora.msg.payloadLength = LoRa.read();

    if (lora.msg.payloadLength > PAYLOAD_SIZE) { lora.err = ERR_PAYLOAD_EXCEDES_BUFFER; return; }

    receivedBytes = 0;
    while (LoRa.available() && (receivedBytes < uint8_t(sizeof(lora.msg.payload) - 1))) lora.msg.payload[receivedBytes++] = LoRa.read();

    if (lora.msg.payloadLength != receivedBytes) { lora.err = ERR_PAYLOAD_NOT_COINCIDES; return; }
    if ((lora.msg.rcpt & lora.localAddr) != lora.localAddr) { lora.err = ERR_TARGET_ERROR; return; }
    if (LoRa.available() && (LoRa.read() != END_SEGMENT)) { lora.msg.endReceived = false; lora.err = ERR_END_NOT_RECEIVED; return; }

    lora.err = NO_ERROR;
    lora.msg.rssi = uint8_t(CONST_PROP_RSSI * LoRa.packetRssi());
    lora.msg.snr = uint8_t(CONST_PROP_SNR + LoRa.packetSnr());

    if (lora.msg.snr >= lora.msg.rssi * SNR_RSSI_RATIO) lora.err = ERR_NOISE_EXCEDES_SIGNAL;

    if (onReceive_call) onReceive_call(lora.msg);
}

void TxFinished() { txDoneFlag = true; }

void lora_receive() {
    LoRa.receive();
}