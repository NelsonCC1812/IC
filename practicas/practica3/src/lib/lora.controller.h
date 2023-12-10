// *=> imports
#include <LoRa.h>

// *=> defines
// *=> consts
const uint8_t syncWord = 0x22;

#define FREC_BAND 868E6 // ICM band; because of law
#define BROADCAST_ADDR 0xff
#define PAYLOAD_SIZE 10
#define END_SEGMENT  0xFF

// *=> srn & rssi
#define SRN_MIN_GAP  1
#define SRN_MAX_RATIO .1
#define RSSI_MIN_PERCENT .1

// *=> duty cycle

#define TX_LAPSE_MS 10000
#define DUTY_CYCLE_MAX 1.0f // because of law
// TODO: maybe this is not needed
// #define DUTY_CYCLE_MIN 0.9f

// *=> errors
#define NO_ERROR 0
#define ERR_END_NOT_RECEIVED 1
#define ERR_PAYLOAD_NOT_COINCIDES 2
#define ERR_NOISE_EXCEDES_SIGNAL 3
#define ERR_TARGET_ERROR 4
#define ERR_PAYLOAD_EXCEDES_BUFFER 99

// operation code
#define OPCODE_CONTROL 0
#define OPCODE_ACK 1
#define OPCODE_NACK 2
#define OPCODE_DATA 3


typedef struct {
    uint8_t bandwidth_index;    // [0-9]
    uint8_t spreadingFactor;    // [6-12] 6 is a special value
    uint8_t codingRate;         // [5-8]
    uint8_t txPower;            // [2-20]
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


// lora struc methods
bool _init(uint8_t localAddr, void (*onReceive_func) (LoraMessage_t msg));
bool _applyConfig(LoraConfig_t config, byte configMask);
bool _sendMessage(uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength);
LoraConfig_t _extractConfig(byte configMask);
void _onReceive(LoraMessage_t message);
bool _receive();
bool _isValidConfig(LoraConfig_t config, byte configMask);

typedef struct {
    // fields
    uint8_t err = 0; // error code
    uint8_t localAddr;
    uint16_t msgCount = 0;
    LoraMessage_t msg;

    // methods
    bool (*init)(uint8_t localAddr, void  (*onReceive_func) (LoraMessage_t)) = _init;
    LoraConfig_t(*extractConfig)(byte configMask) = _extractConfig;
    bool (*applyConfig) (LoraConfig_t config, byte configMask) = _applyConfig;
    bool (*sendMessage) (uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength) = _sendMessage;
    bool (*receive) () = _receive;
    bool (*isValidConfig)(LoraConfig_t config, byte configMask) = _isValidConfig;

} lora_t;
