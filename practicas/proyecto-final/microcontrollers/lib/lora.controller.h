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
#define MESSAGE_DELAY_MS            500
#define CONNECTION_TRY_TIMEOUT_MS   100
#define LORA_RECEIVE_WAITING_MS     10
#define CONNECTION_TRY_TIMES        5

// *=> config mode
#define CM_BW_MASK      0b1
#define CM_SPF_MASK     0b10
#define CM_CR_MASK      0b100
#define CM_PWR_MASK     0b1000
#define CM_CONFS_MASK   0b1111

// life pulse
#define LP_PERIOD_MS 5000 // cada cuanto se pregunta
#define LP_NOCONN_MS 10000 // cuando se considera que no se conecta
#define LP_RETRY_TIMES 5 // Cuantas veces se intenta conectar con una configuración
#define LP_WAISTED_TIME 120000 // el nodo no esta


// *=> srn & rssi
#define SNR_MIN_GAP     1
#define SNR_MAX          0
#define RSSI_MIN_GAP    .1

// *=> duty cycle

#define TX_LAPSE_MS 1000
#define DUTY_CYCLE_MAX 1.0f // because of law


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
    uint32_t lp_lastConn;
    LoraMessage_t msg;
} Node_t;


// lora struc methods
bool _init(uint8_t localAddr, void (*onReceive_func) (LoraMessage_t msg));
bool _sendMessage(uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength, bool waitsForAck);
bool _receive();

void _resetConfig();
bool _sendConfig(uint8_t destAddr, LoraConfig_t config, byte configMask);
bool _applyConfig(LoraConfig_t config, byte configMask);
bool _discover();
bool _reqConfig(uint8_t masterAddr);
void _lifePulseTest();



typedef struct {
    // fields
    uint8_t localAddr;
    uint16_t msgCount = 0;
    volatile bool isReceiving = false;
    bool autoReceive = false;

    LoraConfig_t lastConfig;
    LoraConfig_t config;
    std::map<uint8_t, Node_t> nodes;

    bool hasDynamicConfig = false;
    bool canSendConfig = false;

    // methods
    bool (*init)(uint8_t localAddr, void  (*onReceive_func) (LoraMessage_t)) = _init;
    bool (*sendMessage) (uint8_t destAddr, uint8_t opCode, uint8_t* payload, uint8_t payloadLength, bool waitsForAck) = _sendMessage;
    bool (*receive) () = _receive;

    void (*resetConfig)() = _resetConfig;
    bool (*sendConfig)(uint8_t destAddr, LoraConfig_t config, byte configMask) = _sendConfig;
    bool (*applyConfig) (LoraConfig_t config, byte configMask) = _applyConfig;
    bool (*discover)() = _discover;
    // not used
    bool (*reqConfig)(uint8_t masterAddr) = _reqConfig;
    void (*lifePulseTest)() = _lifePulseTest;

} lora_t;


const LoraConfig_t BASE_CONFIG = { 0, 7, 5, 12 };