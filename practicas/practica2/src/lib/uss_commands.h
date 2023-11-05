/**
 *  USS (Ultra Sound Sensor) => SRF02
*/

// adresses
#define USS_ADDR_0 byte(0x00)
#define USS_ADDR_1 byte(0x01)

#define USS_COMMAND_REGISTER byte(0x00)
#define USS_INIT_DELAY 100
#define USS_RANGING_DELAY 70

// USS data
#define USS_SOFTWARE_REVISION byte(0x00)
#define USS_RANGE_HIGH_BYTE byte(2)
#define USS_RANGE_LOW_BYTE byte(3)
#define USS_AUTOTUNE_MINIMUM_HIGH_BYTE byte(4)
#define USS_AUTOTUNE_MINIMUM_LOW_BYTE byte(5)

// USS comand codes
#define USS_REAL_RANGING_MODE_INCHES    byte(80)
#define USS_REAL_RANGING_MODE_CMS       byte(81)
#define USS_REAL_RANGING_MODE_USECS     byte(82)
#define USS_FAKE_RANGING_MODE_INCHES    byte(86)
#define USS_FAKE_RANGING_MODE_CMS       byte(87)
#define USS_FAKE_RANGING_MODE_USECS     byte(88)
#define USS_TRANSMIT_8CYCLE_40KHZ_BURST byte(92)
#define USS_FORCE_AUTOTUNE_RESTART      byte(96)
#define USS_ADDRESS_CHANGE_1ST_SEQUENCE byte(160)
#define USS_ADDRESS_CHANGE_3RD_SEQUENCE byte(165)
#define USS_ADDRESS_CHANGE_2ND_SEQUENCE byte(170)

// functions
void uss_writeCommand(byte addr, byte command);
byte uss_readRegister(byte addr, byte the_register);