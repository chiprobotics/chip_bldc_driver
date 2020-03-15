#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

uint16_t crc_calc(uint16_t sum, const uint8_t* p, uint32_t len);


#endif // CRC16_H