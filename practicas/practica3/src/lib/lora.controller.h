// *=> imports
#include <LoRa.h>

// *=> defines
#define FREC_BAND 868E6
#define BROADCAST_ADDR 0xff
#define PAYLOAD_SIZE 10
#define END_SEGMENT  0xFF

#define SRN_MIN_GAP  1
#define SRN_MAX_RATIO .1
#define SRN_MIN (x)(10-2.5*x*SRN_MIN_GAP)
#define SNR_MAX (x)((10-2.5*x)*SRN_MAX_RATIO)

#define RSSI_MIN_PERCENT .1
#define RSSI_MIN -120
#define RSSI_MAX -40

#define SNR_RSSI_RATIO .5

// *=> errors
#define NO_ERROR 0
#define ERR_END_NOT_RECEIVED 1
#define ERR_PAYLOAD_NOT_COINCIDES 2
#define ERR_NOISE_EXCEDES_SIGNAL 3
#define ERR_TARGET_ERROR 4
#define ERR_PAYLOAD_EXCEDES_BUFFER 99

// *=> system constants
#define CONST_PROP_RSSI -2
#define CONST_PROP_SNR 148

// *=> consts
const uint8_t syncWord = 0x22;

typedef struct {
    uint8_t bandwidth_index;
    uint8_t spreadingFactor;
    uint8_t codingRate;
    uint8_t txPower;
} LoraConfig_t;


typedef struct {
    uint8_t rcpt, sender;
    uint16_t id;
    uint8_t opCode;

    uint8_t payloadLength;
    uint8_t payload[PAYLOAD_SIZE];

    uint8_t rssi, snr;

    bool endReceived;
} LoraMessage_t;

typedef struct {
    // fields
    uint8_t err = 0; // error code
    uint8_t localAddr, destAddr;
    uint16_t msgCount = 0;
    LoraMessage_t msg;

    // methods
    bool (*init)(uint8_t localAddr, uint8_t destAddr, void  (*onReceive_func) (LoraMessage_t)) = init;
    LoraConfig_t(*extractConfig)(byte configMask) = extractConfig;
    void (*applyConfig) (LoraConfig_t config) = applyConfig;
    void (*sendMessage) (uint8_t opCode, uint8_t* payload, uint8_t payloadLength) = sendMessage;
    void (*receive) () = receive;

} lora_t;

bool init(uint8_t localAddr, uint8_t destAddr, void (*onReceive_func) (LoraMessage_t msg));
void applyConfig(LoraConfig_t config);
void sendMessage(uint8_t opCode, uint8_t* payload, uint8_t payloadLength, uint16_t msgCount);
LoraConfig_t extractConfig(byte configMask);
void onReceive(LoraMessage_t message);
void receive();



// *=> structs
/**
 * LoRaConfig_t
 * lora
*/