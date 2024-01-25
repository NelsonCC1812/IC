// *=> imports
#include <arduino.h>

#include <SPI.h>             
#include <LoRa.h>

#include "lora.controller.h"


// declare

// srn & rssi
uint8_t SNR_REAL_MIN_DB(uint8_t spf) { return 10 - 2.5 * spf; }
uint8_t SNR_MIN(uint8_t spf) { return (10 - 2.5 * spf) * SNR_MIN_GAP; }

#define RSSI_MIN_REAL -120
#define RSSI_MIN()(RSSI_MIN_REAL*RSSI_MIN_GAP)
#define RSSI_MAX -40

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

// *=> system constants
#define CONST_PROP_RSSI -2
#define CONST_PROP_SNR 148

#define BW_SIZE 10


// *=> consts
void (*onReceive_call)(LoraMessage_t);
const double bandwidth_kHz[BW_SIZE] = { 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };


// *=> vars
volatile bool transmitting = false;

uint8_t receivedBytes;
uint8_t currentNode;

uint8_t lora_tmp8;
uint16_t lora_tmp16;
uint8_t total_bits;
uint8_t config_payload[2];

volatile uint32_t lastSendTime_ms = 0;
volatile uint32_t txInterval_ms = TX_LAPSE_MS;
uint32_t tx_begin_ms;
uint32_t TxTime_ms;
uint32_t lapse_ms;
float duty_cycle;

uint32_t connection_try_time_ms;

uint8_t payload[PAYLOAD_SIZE];

uint32_t lp_lastPulseSended_ms = 0;
LoraConfig_t tmpConfig;

lora_t lora;

// *=> headers (private)
bool trySendMessage(uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength);

void onReceive(int packetSize);
void TxFinished();

LoraConfig_t extractConfig(byte payload[PAYLOAD_SIZE], byte configMask);
bool isValidConfig(LoraConfig_t config, byte configMask);
LoraConfig_t refitConfig(uint8_t node);

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

    if (lora.autoReceive) LoRa.receive();

    return true;
}

bool _sendMessage(uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength, bool waitsForAck) {

    if (destAddr == BROADCAST_ADDR)  waitsForAck = false;

    lora.nodes[destAddr].ack = false;


    if (!waitsForAck) {
        lora.nodes[destAddr].waitingAck = false;
        trySendMessage(destAddr, opCode, payload, payloadLength);
        lora.msgCount++;
        return false;
    }

    lora.nodes[destAddr].waitingAck = true;

    for (int i = 0; i < CONNECTION_TRY_TIMES; i++) {

        trySendMessage(destAddr, opCode | OPBIT_ACK_WAITING, payload, payloadLength);

        while (!lora.autoReceive && lora.receive()) {}
        //while (lora.autoReceive && !lora.isReceiving) {}

        connection_try_time_ms = millis();
        while ((millis() - connection_try_time_ms) < CONNECTION_TRY_TIMEOUT_MS) {
            if (lora.nodes[destAddr].ack) {
                lora.msgCount++;
                return true;
            }

            delay(LORA_RECEIVE_WAITING_MS);
        }
    }

    if (lora.autoReceive) lora.receive();
    return false;
}

bool _receive() {
    if (transmitting) return false;
    if (lora.isReceiving) return true;

    LoRa.receive();
    lora.isReceiving = true;

    return true;
}

void _resetConfig() {
    lora.applyConfig(BASE_CONFIG, OPBIT_CONFIG | CM_CONFS_MASK);
}

bool _applyConfig(LoraConfig_t config, byte configMask) {

    lora.lastConfig = lora.config;
    lora.config = config;

    if (!isValidConfig(config, configMask)) return false;

    if (configMask & CM_BW_MASK)   LoRa.setSignalBandwidth(long(bandwidth_kHz[config.bandwidth_index]));
    if (configMask & CM_SPF_MASK)  LoRa.setSpreadingFactor(config.spreadingFactor);
    if (configMask & CM_CR_MASK)   LoRa.setCodingRate4(config.codingRate);
    if (configMask & CM_PWR_MASK)  LoRa.setTxPower(config.txPower, PA_OUTPUT_PA_BOOST_PIN);

    if (lora.autoReceive) LoRa.receive();

    return true;
}


bool _sendConfig(uint8_t destAddr, LoraConfig_t config, byte configMask) {
    lora_tmp16 = 0;
    total_bits = 0;


    if (CM_BW_MASK & configMask) { lora_tmp16 |= (config.bandwidth_index); total_bits += BW_BITS; }
    if (CM_SPF_MASK & configMask) { lora_tmp16 <<= SPF_BITS; lora_tmp16 |= (config.spreadingFactor - SPF_MIN); total_bits += SPF_BITS; }
    if (CM_CR_MASK & configMask) { lora_tmp16 <<= CR_BITS; lora_tmp16 |= (config.codingRate - CR_MIN); total_bits += CR_BITS; }
    if (CM_PWR_MASK & configMask) { lora_tmp16 <<= PWR_BITS; lora_tmp16 |= (config.txPower - PWR_MIN); total_bits += PWR_BITS; }

    lora_tmp8 = (total_bits > 8 ? 2 : 1);

    switch (lora_tmp8) {
    case 1: config_payload[0] = uint8_t(lora_tmp16); break;
    case 2:
        config_payload[0] = uint8_t(lora_tmp16 >> 8);
        config_payload[1] = uint8_t(lora_tmp16);
        break;
    }


    return lora.sendMessage(destAddr, OPBIT_CONFIG | configMask, config_payload, lora_tmp8, OPBIT_ACK_WAITING);
}

bool _discover() {

    uint8_t nodes_quantity = lora.nodes.size();

    for (int i = PWR_MIN; i <= PWR_MAX; i++) {
        payload[0] = i;
        lora.sendMessage(BROADCAST_ADDR, OPCODE_DISCOVER | OPBIT_ACK_WAITING, payload, 1, true);
    }


    return nodes_quantity != lora.nodes.size() ? true : false;
}

bool _reqConfig(uint8_t masterAddr) {

    if (!lora.sendMessage(masterAddr, OPCODE_REQCONFIG | OPBIT_ACK_WAITING, NULL, 0, true)) return false;

    byte configMask = lora.nodes[masterAddr].msg.opCode & (CM_CONFS_MASK | OPBIT_CONFIG);
    LoraConfig_t config = extractConfig(lora.nodes[masterAddr].msg.payload, configMask);

    return _applyConfig(config, configMask);
}

// private

LoraConfig_t extractConfig(byte payload[PAYLOAD_SIZE], byte configMask) {
    LoraConfig_t config;

    lora_tmp8 = 0;


    if (configMask & CM_BW_MASK)   lora_tmp8 += BW_BITS;
    if (configMask & CM_SPF_MASK)  lora_tmp8 += SPF_BITS;
    if (configMask & CM_CR_MASK)   lora_tmp8 += CR_BITS;
    if (configMask & CM_PWR_MASK)  lora_tmp8 += PWR_BITS;

    lora_tmp8 = (lora_tmp8 > 8 ? 2 : 1);


    switch (lora_tmp8) {
    case 1: lora_tmp16 = (uint16_t)payload[0]; break;
    case 2: lora_tmp16 = (uint16_t)((payload[1]) | uint16_t(payload[0] << 8)); break;
    }


    if (configMask & CM_PWR_MASK) { config.txPower = (lora_tmp16 & PWR_MASK) + PWR_MIN; lora_tmp16 >>= PWR_BITS; }
    if (configMask & CM_CR_MASK) { config.codingRate = (lora_tmp16 & CR_MASK) + CR_MIN; lora_tmp16 >>= CR_BITS; }
    if (configMask & CM_SPF_MASK) { config.spreadingFactor = (lora_tmp16 & SPF_MASK) + SPF_MIN; lora_tmp16 >>= SPF_BITS; }
    if (configMask & CM_BW_MASK) { config.bandwidth_index = lora_tmp16 & BW_MASK; }


    return config;
}

void onReceive(int packetSize) {
    if (packetSize == 0) return;

    currentNode = LoRa.read();

    lora.nodes[currentNode].msg.sender = currentNode;
    lora.nodes[currentNode].msg.rcpt = LoRa.read();

    lora.nodes[currentNode].msg.id = ((uint16_t)(LoRa.read() << 8) | (uint16_t)LoRa.read());
    lora.nodes[currentNode].msg.opCode = LoRa.read();

    lora.nodes[currentNode].msg.payloadLength = LoRa.read();

    lora.nodes[currentNode].lp_lastConn = millis();

    if (lora.nodes[currentNode].msg.payloadLength > PAYLOAD_SIZE) { lora.nodes[currentNode].err = ERR_PAYLOAD_EXCEDES_BUFFER; return; }

    receivedBytes = 0;
    while (LoRa.available() && (receivedBytes < lora.nodes[currentNode].msg.payloadLength)) lora.nodes[currentNode].msg.payload[receivedBytes++] = LoRa.read();

    if (lora.nodes[currentNode].msg.payloadLength != receivedBytes) { lora.nodes[currentNode].err = ERR_PAYLOAD_NOT_COINCIDES; return; }
    if ((lora.nodes[currentNode].msg.rcpt & lora.localAddr) != lora.localAddr) { lora.nodes[currentNode].err = ERR_TARGET_ERROR; return; }
    if (LoRa.available() && (LoRa.read() != END_SEGMENT)) { lora.nodes[currentNode].msg.endReceived = false; lora.nodes[currentNode].err = ERR_END_NOT_RECEIVED; return; }

    lora.nodes[currentNode].err = NO_ERROR;

    lora.nodes[currentNode].msg.rssi = uint8_t(CONST_PROP_RSSI * LoRa.packetRssi());
    lora.nodes[currentNode].msg.snr = uint8_t(CONST_PROP_SNR + LoRa.packetSnr());


    if (lora.nodes[currentNode].msg.snr <= SNR_REAL_MIN_DB(lora.config.spreadingFactor))
        lora.nodes[currentNode].err = ERR_NOISE_EXCEDES_SIGNAL;


    if (lora.nodes[currentNode].msg.opCode == OPCODE_ACK && lora.nodes[currentNode].waitingAck) {
        lora.nodes[currentNode].waitingAck = false;
        lora.nodes[currentNode].ack = true;
    }

    if (lora.nodes[currentNode].msg.opCode == OPCODE_DISCOVER) {
        LoraConfig_t config;
        config.txPower = lora.nodes[currentNode].msg.payload[0];

        lora.applyConfig(config, CM_PWR_MASK);
    }

    if (lora.hasDynamicConfig && lora.nodes[currentNode].msg.opCode & OPBIT_CONFIG)
        lora.applyConfig(extractConfig(lora.nodes[currentNode].msg.payload, lora.nodes[currentNode].msg.opCode), lora.nodes[currentNode].msg.opCode);


    if (lora.nodes[currentNode].msg.opCode & OPBIT_ACK_WAITING) {

        payload[0] = (uint8_t)(lora.nodes[currentNode].msg.id >> 8);
        payload[1] = (uint8_t)(lora.nodes[currentNode].msg.id);

        lora.sendMessage(currentNode, OPCODE_ACK, payload, 2, false);
    }

    if (onReceive_call) onReceive_call(lora.nodes[currentNode].msg);

    // configs fit
    if (lora.hasDynamicConfig && lora.canSendConfig) {
        tmpConfig = refitConfig(currentNode);
        if (!tmpConfig.txPower) return;
        if (lora.sendConfig(currentNode, tmpConfig, CM_CONFS_MASK)) lora.applyConfig(tmpConfig, CM_CONFS_MASK);
        if (!lora.sendMessage(currentNode, OPCODE_PING | OPBIT_ACK_WAITING, NULL, 0, true)) lora.applyConfig(lora.lastConfig, CM_CONFS_MASK);
    }
}

void TxFinished() {
    if (!transmitting) return;

    transmitting = false;

    TxTime_ms = millis() - tx_begin_ms;
    lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms;

    duty_cycle = (100.0f * TxTime_ms) / lapse_ms;


    if (duty_cycle > DUTY_CYCLE_MAX) txInterval_ms = TxTime_ms * DUTY_CYCLE_MAX * 100;
    lora.autoReceive&& lora.receive();
}

bool trySendMessage(uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength) {

    while (transmitting) { //|| ((millis() - lastSendTime_ms) < txInterval_ms)) {
        delay(MESSAGE_DELAY_MS);
    }



    while (!LoRa.beginPacket()) delay(10);

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

bool isValidConfig(LoraConfig_t config, byte configMask) {

    if ((CM_BW_MASK & configMask) && (config.bandwidth_index < BW_MIN || config.bandwidth_index > BW_MAX)) return false;
    if ((CM_SPF_MASK & configMask) && (config.spreadingFactor < SPF_MIN || config.spreadingFactor > SPF_MAX)) return false;
    if ((CM_CR_MASK & configMask) && (config.codingRate < CR_MIN || config.codingRate > CR_MAX)) return false;
    if ((CM_PWR_MASK & configMask) && (config.txPower < PWR_MIN || config.txPower > PWR_MAX)) return false;

    return true;
}

LoraConfig_t refitConfig(uint8_t node) {

    bool hasBeingModified = false;

    LoraMessage_t msg = lora.nodes[node].msg;
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


    if (msg.snr <= SNR_MIN(lora.config.spreadingFactor)) {
        hasBeingModified = true;

        if (config.bandwidth_index <= BW_MIN) return (LoraConfig_t) { 0, 0, 0, 0 };

        config.bandwidth_index--;
        config.spreadingFactor = calcSpfThroughtBW(config.bandwidth_index);
    }

    if (msg.snr > SNR_MAX) {
        hasBeingModified = true;

        if (config.bandwidth_index >= BW_MAX) return (LoraConfig_t) { 0, 0, 0, 0 };

        config.bandwidth_index++;
        config.spreadingFactor = calcSpfThroughtBW(config.bandwidth_index);
    }


    return  hasBeingModified ? config : (LoraConfig_t) { 0, 0, 0, 0 };
}


uint8_t calcSpfThroughtBW(uint8_t bw_index) {
    return  ((-bw_index + BW_SIZE + 1) / BW_SIZE) * (SPF_MAX - SPF_MIN) + SPF_MIN;
}


void _lifePulseTest() {

    for (const auto& [key, value] : lora.nodes) { // https://www.delftstack.com/es/howto/cpp/how-to-iterate-over-map-in-cpp/

        // comprobacion de lp
        if ((millis() - value.lp_lastConn) <= LP_PERIOD_MS) break;
        if (lora.sendMessage(key, OPCODE_PING, NULL, 0, true)) break;

        // Si la ultima conexion fue hace mucho, se reintenta un numero de veces
        if ((millis() - value.lp_lastConn) < LP_NOCONN_MS) break;

        for (int i = 0; i < LP_RETRY_TIMES; i++) if (lora.sendMessage(key, OPCODE_PING, NULL, 0, true))  break;

        if (lora.hasDynamicConfig) lora.resetConfig();

        for (int i = 0; i < LP_RETRY_TIMES; i++) if (lora.sendMessage(key, OPCODE_PING, NULL, 0, true))  break;

        // si ha pasado demasiado tiempo se considera que el nodo no esta
        if ((millis() - value.lp_lastConn) > LP_WAISTED_TIME) lora.nodes.erase(key);
    }
}


/** TODO:
 * Ahora mismo estas cambiando la configuracion antes de mandar el mensaje de cambio de configuracion
 * tienes que mandar el paquete de cambbio de configuracion y luego modificar la configuracion
 *
*/