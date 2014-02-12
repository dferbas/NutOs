int OWTouchReset(void);
int OWInit(void);
int OWRomSearch(uint8_t diff, uint64_t *hid);
int OWReadBit(void);
void OWWriteBit(int bit);
uint8_t OWReadByte(void);
int w1_command( uint8_t command, uint64_t *id );

#define SEARCH_ROM      0xF0
#define LAST_DEVICE     0x00
#define DATA_ERR        0xFE
#define PRESENCE_ERR    0xFF
#define SEARCH_FIRST    0xFF
#define READ            0xBE
#define MATCH_ROM       0x55
#define SKIP_ROM        0xCC
#define CONVERT_T       0x44            // DS1820 commands
#define READ_ROM        0x33

#define TEMP_GENERAL_ERROR  0xffff
#define TEMP_PRESENCE_ERR   0Xfffe
#define TEMP_BUS_ERROR      0xfffd
