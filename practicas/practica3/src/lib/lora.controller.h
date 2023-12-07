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
const uint8_t syncWord = 0x22


// *=> structs
/**
 * LoRaConfig_t
 * lora
*/