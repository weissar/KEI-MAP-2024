#ifndef _MBED_I2C_H
#define _MBED_I2C_H

#include "stm_core.h"

#define MAX_TIMEOUT   10000

typedef enum {i2cSpeed100k = 100000, i2cSpeed400k = 400000} i2cSpeed;
bool InitI2C1(i2cSpeed);
bool I2C1_WriteByte(uint8_t devAdr, uint8_t regAdr, uint8_t val);
uint8_t I2C1_ReadByte(uint8_t devAdr, uint8_t regAdr);
bool I2C1_ReadBytes(uint8_t devAdr, uint8_t regAdr, uint8_t *pbuf, uint32_t len);

#endif // _MBED_I2C_H
