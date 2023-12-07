// *=> imports
#include <arduino.h>
#include <SPI.h>             
#include <LoRa.h>

#include <lora.controller.h>


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


// *=> lora headers
void init(uint8_t localAddr, uint8_t destAddr, uint8_t onReceive_func);
void applyConfig(LoraConfig_t config);
void sendMessage(uint8_t opCode, uint8_t* payload, uint8_t payloadLength, uint16_t msgCount);
LoRaConfig_t extractConfig(byte configMask);
void onReceive(LoraMessage_t message);

// *=> structs

typedef struct {
    uint8_t bandwidth_index;
    uint8_t spreadingFactor;
    uint8_t codingRate;
    uint8_t txPower;
} LoRaConfig_t;

typedef struct {
    uint8_t rcpt, sender;
    uint16_t id;
    uint8_t opCode;

    uint8_t payloadLength;
    uint8_t payload[PAYLOAD_SIZE];

    uint8_t rssi, snr;

    bool endRecieved;
} LoraMessage_t;

struct {
    // fields
    uint8_t err = 0; // error code
    uint8_t localAddr, destAddr;
    uint16_t msgCount = 0;
    LoraMessage_t msg;

    // methods
    void (*init)(uint8_t localAddr, uint8_t destAddr, uint8_t onReceive_func) = init;
    LoRaConfig_t(*extractConfig)(byte configMask) = extractConfig;
    void (*applyConfig) (LoRaConfig_t config) = applyConfig;
    void (*sendMessage) (uint8_t opCode, uint8_t* payload, uint8_t payloadLength, uint16_t msgCount) = sendMessage;
    void (*receive) () = LoRa.receive;

} lora;


// *=> implementations

bool init(uint8_t localAddr, uint8_t destAddr, uint8_t onReceive_func) {

    lora.localAddr = localAddr;
    lora.destAddr = destAddr;

    onReceive_call = onReceive_func;

    if (!LoRa.begin(FREC_BAND)) return false;

    LoRa.setSyncWord(syncWord);
    LoRa.setPreambleLength(8);
    LoRa.onReceive(onReceive);
    LoRa.onTxDone(TxFinished);
    Lora.receive();

    return true;
}

LoRaConfig_t extractConfig(byte configMask) {
    if (!(configMask & CONF_MODE_MASK)) return;

    lora_tmp = 0;

    if (configMask & BW_MASK)   lora_tmp++;
    if (configMask & SPF_MASK)  lora_tmp++;
    if (configMask & CR_MASK)   lora_tmp++;
    if (configMask & PWR_MASK)  lora_tmp++;

    if (lora.msg.payloadLength > lora_tmp || lora.msg.payloadLength == 0) return NULL;

    LoRaConfig_t config;
    lora_tmp = 0;

    if (configMask & BW_MASK)   config.bandwidth_index = lora.msg.payload[lora_tmp++];
    if (configMask & SPF_MASK)  config.spreadingFactor = lora.msg.payload[lora_tmp++];
    if (configMask & CR_MASK)   config.codingRate = lora.msg.payload[lora_tmp++];
    if (configMask & PWR_MASK)  config.txPower = lora.msg.payload[lora_tmp++];

    return config;
}

void applyConfig(LoRaConfig_t config, byte configMask) {
    if (!(configMask & CONF_MODE_MASK)) return;

    if (configMask & BW_MASK)   LoRa.setSignalBandwidth(long(bandwidth_kHz[config.bandwidth_index]));
    if (configMask & SPF_MASK)  LoRa.setSpreadingFactor(config.spreadingFactor);
    if (configMask & CR_MASK)   LoRa.setCodingRate4(config.codingRate);
    if (configMask & PWR_MASK)  LoRa.setTxPower(config.txPower, PA_OUTPUT_PA_BOOST_PIN);
}

void sendMessage(uint8_t opCode, uint8_t* payload, uint8_t payloadLength, uint16_t msgCount) {

    transmitting = true;
    txDoneFlag = false;

    while (!LoRa.beginPacket()) delay(10);

    LoRa.write(lora.destAddr);
    LoRa.write(lora.localAddr);

    LoRa.write((uint8_t)(msgCount >> 8));
    LoRa.write((uint8_t)(msgCount & 0xFF));

    Lora.write(opCode);

    LoRa.write(payloadLength);
    LoRa.write(payload, (size_t)payloadLength);

    LoRa.write(END_SEGMENT);

    LoRa.endPacket(true);
}


void onReceive(int packetSize) {
    if (transmitting && !txDoneFlag) txDoneFlag = true;
    if (packetSize == 0) return;

    lora.msg.rcpt = LoRa.read();
    lora.msg.sender = LoRa.read();

    lora.msg.id = ((uint16_t)(LoRa.read() << 8) | (uint16_t)LoRa.read());
    lora.msg.opCode = Lora.read();

    lora.msg.payloadLength = LoRa.read();

    if (lora.msg.payloadLength > PAYLOAD_SIZE) { lora.err = ERR_PAYLOAD_EXCEDES_BUFFER; return; }

    receivedBytes = 0;
    while (LoRa.available() && (receivedBytes < uint8_t(sizeof(lora.msg.payload) - 1))) buff[receivedBytes++] = LoRa.read();

    if (lora.msg.payloadLength != receivedBytes) { lora.err = ERR_PAYLOAD_NOT_COINCIDES; return; }
    if ((lora.msg.rcpt & lora.localAddr) != lora.localAddr) { lora.err = ERR_TARGET_ERROR; return; }
    if (LoRa.available() && (LoRa.read() != END_SEGMENT)) { lora.msg.endRecieved = false; lora.err = ERR_END_NOT_RECEIVED; return; }

    lora.err = NO_ERROR;
    lora.msg.rssi = uint8_t(CONST_PROP_RSSI * LoRa.packetRssi());
    lora.msg.snr = uint8_t(CONST_PROP_SNR + LoRa.packetSnr());

    if (lora.msg.snr >= lora.msg.rssi * SNR_RSSI_RATIO) lora.err = ERR_NOISE_EXCEDES_SIGNAL;

    if (onReceive_call) onReceive_call(lora.msg);
}

void TxFinished() { txDoneFlag = true; }
