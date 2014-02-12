int OWTouchReset(void);
int OWInit(void);
int OWRomSearch(uint8_t diff, uint64_t *hid);
int OWReadBit(void);
void OWWriteBit(int bit);
uint8_t OWReadByte(void);
int w1_command( uint8_t command, uint64_t *id );
