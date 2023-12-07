// *=> defines
#define FREC_BAND 868E6
#define BROADCAST_ADDR 0xff
#define PAYLOAD_SIZE 10


#define SRN_MIN_GAP  1
#define SRN_MAX_RATIO .1
#define SRN_MIN (x)(10-2.5*x*SRN_MIN_GAP)
#define SNR_MAX (x)((10-2.5*x)*SRN_MAX_RATIO)

#define RSSI_MIN_PERCENT .1
#define RSSI_MIN -120
#define RSSI_MAX -40

#define SNR_RSSI_RATIO .5

// *=> consts
const uint8_t syncWord = 0x22;
const uint8_t END_SEGMENT = 0xFF;


// *=> structs
/**
 * LoRaConfig_t
 * lora
*/