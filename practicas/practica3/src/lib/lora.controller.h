// *=> imports
#include <map>

#include <LoRa.h>


// *=> consts
const uint8_t syncWord = 0x22;

// *=> defines
#define FREC_BAND       868E6 // ICM band; because of law
#define BROADCAST_ADDR  0xff
#define END_SEGMENT     0xFF
#define PAYLOAD_SIZE    10

#define CONNECTION_TRY_TIMES 5
#define CONNECTION_TRY_TIMEOUT_MS 500
#define REVERT_CONFIG_TIMEOUT 20000
#define REVERT_BASE_CONFIG_TIMEOUT 60000

// *=> srn & rssi
#define SNR_MIN_GAP     1
#define SNR_MAX          5
#define RSSI_MIN_GAP    .4

// *=> duty cycle
#define TX_LAPSE_MS 1000
#define DUTY_CYCLE_MAX 1.0f


// *=> operation code
// operation bits
#define OPBIT_CONFIG        0b10000000
#define OPBIT_ACK_WAITING   0b01000000
#define OPCODE_ACK          1
#define OPCODE_PING         30

// *=> errors
#define NO_ERROR                    0
#define ERR_END_NOT_RECEIVED        1
#define ERR_PAYLOAD_NOT_COINCIDES   2
#define ERR_NOISE_EXCEDES_SIGNAL    3
#define ERR_TARGET_ERROR            4
#define ERR_PAYLOAD_EXCEDES_BUFFER  99


// *=> structs & types

typedef struct {
    uint8_t bandwidth_index;    // [0-9]
    uint8_t spreadingFactor;    // [6-12] 6 is a special value
    uint8_t codingRate;         // [5-8]
    uint8_t txPower;            // [2-20]
} LoraConfig_t;


typedef struct {
    volatile uint8_t rcpt, sender;
    volatile uint16_t id;
    volatile uint8_t opCode;

    volatile uint8_t payloadLength;
    uint8_t payload[PAYLOAD_SIZE];

    volatile float rssi, snr;

    volatile bool endReceived;
    volatile uint8_t err;
} LoraMessage_t;


typedef struct {
    volatile bool ack = false;
    volatile bool waitingAck = false;
    volatile uint32_t lp_lastConn;
    LoraMessage_t msg;
} Node_t;


// lora struc methods
bool _init(uint8_t localAddr, void (*onReceive_func) (LoraMessage_t msg));
bool _sendMessage(uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength);
bool _receive(bool force);

void _resetConfig();
bool _sendConfig(uint8_t destAddr, LoraConfig_t config);
bool _applyConfig(LoraConfig_t config);
void _lifePulseTest();
void _control();



typedef struct {
    // fields
    uint8_t localAddr;
    uint16_t msgCount = 0;
    volatile bool isReceiving = false;

    LoraConfig_t lastConfig;
    LoraConfig_t config;
    bool isMaster = false;
    Node_t node;

    // methods
    bool (*init)(uint8_t localAddr, void  (*onReceive_func) (LoraMessage_t)) = _init;
    bool (*sendMessage) (uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength) = _sendMessage;
    bool (*receive) (bool force) = _receive;

    void (*resetConfig)() = _resetConfig;
    bool (*sendConfig)(uint8_t destAddr, LoraConfig_t config) = _sendConfig;
    bool (*applyConfig) (LoraConfig_t config) = _applyConfig;

    void (*control)() = _control;

} lora_t;


const LoraConfig_t BASE_CONFIG = { 7, 8, 5, 8 };