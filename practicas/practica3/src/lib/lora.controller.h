// *=> imports
#include <map>
#include <LoRa.h>


// *=> consts
const uint8_t syncWord = 0x22;

// *=> defines
#define FREC_BAND       868E6 // ICM band; because of law
#define BROADCAST_ADDR  0xff
#define END_SEGMENT     0xFF
#define MAX_NODES       5
#define PAYLOAD_SIZE    10

// *=> timeouts & trys
#define MESSAGE_DELAY_MS            10
#define CONNECTION_TRY_TIMEOUT_MS   100
#define LORA_RECEIVE_WAITING_MS     10
#define CONNECTION_TRY_TIMES        5


// *=> srn & rssi
#define SRN_MIN_GAP     1
#define SRN_MAX         0
#define RSSI_MIN_GAP    .1

// *=> duty cycle

#define TX_LAPSE_MS 1000
#define DUTY_CYCLE_MAX 1.0f // because of law
// TODO: maybe this is not needed
// #define DUTY_CYCLE_MIN 0.9f


// *=> operation code
// operation bits
#define OPBIT_CONFIG        0b10000000
#define OPBIT_ACK_WAITING   0b1000000

// operation codes
#define OPCODE_CONTROL      0
#define OPCODE_ACK          1
#define OPCODE_NACK         2
#define OPCODE_DATA         3
#define OPCODE_DISCOVER     10
#define OPCODE_REQCONFIG    20
#define OPCODE_SENDCONFIG   21

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
    uint8_t rcpt, sender;
    uint16_t id;
    uint8_t opCode;

    uint8_t payloadLength;
    uint8_t payload[PAYLOAD_SIZE];

    uint8_t rssi, snr;

    bool endReceived;
} LoraMessage_t;


typedef struct {
    bool ack = false;
    bool waitingAck = false;
    uint8_t err = NO_ERROR;
    LoraMessage_t msg;
} Node_t;


// lora struc methods
bool _init(uint8_t localAddr, void (*onReceive_func) (LoraMessage_t msg));
bool _applyConfig(LoraConfig_t config, byte configMask);
bool _sendMessage(uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength, bool waitsForAck);
void _onReceive(LoraMessage_t message);
bool _receive();
bool _discover();
bool _reqConfig(uint8_t masterAddr);


typedef struct {
    // fields
    uint8_t localAddr;
    uint16_t msgCount = 0;
    bool isReceiving = false;

    LoraConfig_t config;
    std::map<uint8_t, Node_t> nodes;

    bool hasDynamicConfig = false;
    bool canSendConfig = false;

    // methods
    bool (*init)(uint8_t localAddr, void  (*onReceive_func) (LoraMessage_t)) = _init;
    bool (*sendMessage) (uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength, bool waitsForAck) = _sendMessage;
    bool (*receive) () = _receive;

    bool (*discover)() = _discover;
    bool (*reqConfig)(uint8_t masterAddr) = _reqConfig;
    bool (*applyConfig) (LoraConfig_t config, byte configMask) = _applyConfig;

} lora_t;


/** // TODO
 * COnfiguración dinamica
*/