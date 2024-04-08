#ifndef DISP_TM1640_H_
#define DISP_TM1640_H_

#include "stm_core.h"   // includeing stdint.h and stdbool.h

bool TM1640_Init(void);

void TM1640_DispOff(void);
void TM1640_DispOn(void);
void TM1640_DispBright(uint8_t b);
void TM1640_Refresh(void);

bool TM1640_SetPixel(uint8_t x, uint8_t y, bool on);
bool TM1640_SetByte(int pos, uint8_t val);

#endif /* DISP_TM1640_H_ */
