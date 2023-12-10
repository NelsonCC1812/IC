// *=> imports
#include <arduino.h>
#include <SPI.h>             
#include <LoRa.h>

#include "lora.controller.h"


// declare

// *=> config mode
#define CONF_MODE_MASK  0b10000000
#define CM_BW_MASK      0b1
#define CM_SPF_MASK     0b10
#define CM_CR_MASK      0b100
#define CM_PWR_MASK     0b1000

// srn & rssi
#define SNR_RSSI_RATIO .5
#define SRN_MIN (x)(10-2.5*x*SRN_MIN_GAP)
#define SNR_MAX (x)((10-2.5*x)*SRN_MAX_RATIO)
#define RSSI_MIN -120
#define RSSI_MAX -40

// *=> config params
#define BW_BITS 4
#define BW_MASK 0b1111
#define BW_MIN 0
#define BW_MAX 9

#define SPF_BITS 3
#define SPF_MASK 0b111
#define SPF_MIN 6
#define SPF_MAX 12

#define CR_BITS 2
#define CR_MASK 0b11
#define CR_MIN 5
#define CR_MAX 8

#define PWR_BITS 5
#define PWR_MASK 0b11111
#define PWR_MIN 2
#define PWR_MAX 20

// *=> system constants
#define CONST_PROP_RSSI -2
#define CONST_PROP_SNR 148


// *=> consts
const double bandwidth_kHz[10] = { 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };
void (*onReceive_call)(LoraMessage_t);

// *=> vars

volatile bool txDoneFlag = true;
volatile bool transmitting = false;

uint8_t receivedBytes;

uint8_t lora_tmp8;
uint16_t lora_tmp16;
uint8_t total_bits;
uint8_t config_payload[2];


uint16_t msgCount = 0;

uint32_t lastSendTime_ms = 0;
uint32_t txInterval_ms = TX_LAPSE_MS;
uint32_t tx_begin_ms;
uint32_t TxTime_ms;
uint32_t lapse_ms;
float duty_cycle;

lora_t lora;

// *=> headers
void TxFinished();
void _onReceive(int packetSize);

// *=> implementations

bool _init(uint8_t localAddr, void (*onReceive_func) (LoraMessage_t msg)) {

    lora.localAddr = localAddr;

    onReceive_call = onReceive_func;

    if (!LoRa.begin(FREC_BAND)) return false;

    LoRa.setSyncWord(syncWord);
    LoRa.setPreambleLength(8);
    LoRa.onReceive(_onReceive);
    LoRa.onTxDone(TxFinished);
    LoRa.receive();

    return true;
}
// TODO
LoraConfig_t _extractConfig(byte configMask) {
    LoraConfig_t config;

    if (!(configMask & CONF_MODE_MASK)) return config;

    lora_tmp8 = 0;

    if (configMask & CM_BW_MASK)   lora_tmp8 += BW_BITS;
    if (configMask & CM_SPF_MASK)  lora_tmp8 += SPF_BITS;
    if (configMask & CM_CR_MASK)   lora_tmp8 += CR_BITS;
    if (configMask & CM_PWR_MASK)  lora_tmp8 += PWR_BITS;

    lora_tmp8 = (total_bits > 8 ? 2 : 1);

    switch (lora_tmp8) {
    case 1: lora_tmp16 = (uint16_t)lora.msg.payload[0]; break;
    case 2: lora_tmp16 = (uint16_t)((lora.msg.payload[0] << 8) | lora.msg.payload[1]); break;
    }

    if (configMask & CM_PWR_MASK) { config.txPower = lora_tmp16 & PWR_MASK; lora_tmp16 >>= PWR_BITS; }
    if (configMask & CM_CR_MASK) { config.codingRate = lora_tmp16 & CR_MASK; lora_tmp16 >>= CR_BITS; }
    if (configMask & CM_SPF_MASK) { config.spreadingFactor = lora_tmp16 & SPF_MASK; lora_tmp16 >>= SPF_BITS; }
    if (configMask & CM_BW_MASK) { config.bandwidth_index = lora_tmp16 & BW_MASK; lora_tmp16 >>= BW_BITS; }

    return config;
}


bool _applyConfig(LoraConfig_t config, byte configMask) {

    if (!(configMask & CONF_MODE_MASK)) return false;
    if (!_isValidConfig(config, configMask)) return false;

    if (configMask & CM_BW_MASK)   LoRa.setSignalBandwidth(long(bandwidth_kHz[config.bandwidth_index]));
    if (configMask & CM_SPF_MASK)  LoRa.setSpreadingFactor(config.spreadingFactor);
    if (configMask & CM_CR_MASK)   LoRa.setCodingRate4(config.codingRate);
    if (configMask & CM_PWR_MASK)  LoRa.setTxPower(config.txPower, PA_OUTPUT_PA_BOOST_PIN);

    return true;
}


void _sendConfig(uint8_t destAddr, LoraConfig_t config, byte configMask) {

    lora_tmp16 = 0;
    total_bits = 0;

    if (CM_BW_MASK & configMask) { lora_tmp16 |= (config.bandwidth_index); lora_tmp16 <<= BW_BITS;  total_bits += BW_BITS; }
    if (CM_SPF_MASK & configMask) { lora_tmp16 |= (config.spreadingFactor - SPF_MIN); lora_tmp16 <<= SPF_BITS; total_bits += SPF_BITS; }
    if (CM_CR_MASK & configMask) { lora_tmp16 |= (config.codingRate - CR_MIN); lora_tmp16 <<= CR_BITS; total_bits += CR_BITS; }
    if (CM_PWR_MASK & configMask) { lora_tmp16 |= (config.txPower - PWR_MIN); lora_tmp16 <<= PWR_BITS; total_bits += PWR_BITS; }

    lora_tmp8 = (total_bits > 8 ? 2 : 1);

    switch (lora_tmp8) {
    case 1: config_payload[0] = uint8_t(lora_tmp16); break;
    case 2:
        config_payload[0] = uint8_t(lora_tmp16 >> 8);
        config_payload[1] = uint8_t(lora_tmp16);
        break;
    }

    lora.sendMessage(destAddr, CONF_MODE_MASK & configMask, config_payload, lora_tmp8);
}


bool _sendMessage(uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength) {

    if (transmitting || ((millis() - lastSendTime_ms) < txInterval_ms)) return false;

    transmitting = true;
    txDoneFlag = false;

    while (!LoRa.beginPacket()) delay(10);

    LoRa.write(destAddr);
    LoRa.write(lora.localAddr);

    LoRa.write((uint8_t)(msgCount >> 8));
    LoRa.write((uint8_t)(msgCount & 0xFF));

    msgCount++;

    LoRa.write(opCode);

    LoRa.write(payloadLength);
    LoRa.write(payload, (size_t)payloadLength);

    LoRa.write(END_SEGMENT);

    LoRa.endPacket(true);

    return true;
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
    while (LoRa.available() && (receivedBytes < lora.msg.payloadLength)) lora.msg.payload[receivedBytes++] = LoRa.read();

    if (lora.msg.payloadLength != receivedBytes) { lora.err = ERR_PAYLOAD_NOT_COINCIDES; return; }
    if ((lora.msg.rcpt & lora.localAddr) != lora.localAddr) { lora.err = ERR_TARGET_ERROR; return; }
    if (LoRa.available() && (LoRa.read() != END_SEGMENT)) { lora.msg.endReceived = false; lora.err = ERR_END_NOT_RECEIVED; return; }

    lora.err = NO_ERROR;
    lora.msg.rssi = uint8_t(CONST_PROP_RSSI * LoRa.packetRssi());
    lora.msg.snr = uint8_t(CONST_PROP_SNR + LoRa.packetSnr());

    if (lora.msg.snr >= lora.msg.rssi * SNR_RSSI_RATIO) lora.err = ERR_NOISE_EXCEDES_SIGNAL;

    if (onReceive_call) onReceive_call(lora.msg);
}


bool _receive() {

    if (!transmitting || !txDoneFlag) return false;

    TxTime_ms = millis() - tx_begin_ms;
    lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms;

    duty_cycle = (100.0f * TxTime_ms) / lapse_ms;


    if (duty_cycle > DUTY_CYCLE_MAX) txInterval_ms = TxTime_ms * DUTY_CYCLE_MAX * 100;
    // TODO: maybe this is not needed
    //if (duty_cycle < DUTY_CYCLE_MIN) txInterval_ms = TxTime_ms * DUTY_CYCLE_MIN * 100;

    LoRa.receive();

    return true;
}

void TxFinished() { txDoneFlag = true; }


bool _isValidConfig(LoraConfig_t config, byte configMask) {

    if ((CM_BW_MASK & configMask) && (config.bandwidth_index < BW_MIN || config.bandwidth_index > BW_MAX)) return false;
    if ((CM_SPF_MASK & configMask) && (config.spreadingFactor < SPF_MIN || config.spreadingFactor > SPF_MAX)) return false;
    if ((CM_CR_MASK & configMask) && (config.codingRate < CR_MIN || config.codingRate > CR_MAX)) return false;
    if ((CM_PWR_MASK & configMask) && (config.txPower < PWR_MIN || config.txPower > PWR_MAX)) return false;

    return true;
}