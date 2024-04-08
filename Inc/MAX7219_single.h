#ifndef MAX7219_SINGLE_H_
#define MAX7219_SINGLE_H_

#include "stm_core.h"

bool MAX7219_InitHW(void);

extern const uint8_t to7seg[16];

void MAX7219_Send16b(uint16_t w);
bool MAX7219_SendDataView(uint8_t pos, uint8_t val);

#endif // MAX7219_SINGLE_H_
