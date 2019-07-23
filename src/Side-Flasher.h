#ifndef SIDE_FLASHER
#define SIDE_FLASHER

#define ENDTRANS_SUCCESS 0
#define ENDTRANS_DATA_TOO_LONG 1
#define ENDTRANS_ADDR_NACK 2
#define ENDTRANS_DATA_NACK 3
#define ENDTRANS_ERROR 4

#define LEFT_BOOT_ADDRESS 0x50
#define RIGHT_BOOT_ADDRESS 0x51

void power_off_attiny();
void power_on_attiny();
int run_command(uint8_t address, uint8_t command);
uint8_t read_crc16(uint8_t addr, uint8_t *version, uint16_t *crc16, uint16_t offset, uint16_t length);
int get_version (uint8_t addr);
int erase_program(uint8_t addr);
int write_firmware(uint8_t addr);
int verify_firmware(uint8_t addr);
uint8_t update_attiny(uint8_t addr);

#endif
