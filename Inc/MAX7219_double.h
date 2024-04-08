#ifndef MAX7219_DOUBLE_H_
#define MAX7219_DOUBLE_H_

#include "stm_core.h"

bool MAX7219_InitHW(void);

void MAX7219_Dbl_Refresh(void);
void MAX7219_Dbl_Clear(void);
bool MAX7219_Dbl_PutPixel(int x, int y, uint32_t color);   // color 0 / non-0
bool MAX7219_Dbl_InitSW(void);
bool MAX7219_Dbl_DrawNum_3x5(int x, int y, char c);

#endif // MAX7219_DOUBLE_H_
