#ifndef CRC_CHECK_HPP
#define CRC_CHECK_HPP

#include <cstdint>
#include <cstddef>

//CRC8
#define CRC_START_8 0x00

uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);

uint8_t update_crc_8(uint8_t crc, uint8_t val);

//CRC16
#define CRC_START_16 0xFFFF
#define CRC_START_MODBUS 0xFFFF
#define CRC_POLY_16 0xA001

uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes);
uint16_t crc_modbus(const uint8_t *input_str, uint16_t num_bytes);
uint16_t update_crc_16(uint16_t crc, uint8_t c);
void init_crc16_tab(void);

#endif