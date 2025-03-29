
#ifndef _HT1621_LCD_H_
#define _HT1621_LCD_H_

#include "stm_core.h"

bool HT1621_Init(int numOfDigits);
void HT1621_WriteDigitData(uint8_t posDigit, uint8_t data);

bool HT1621_WriteDigitChar(uint8_t posDigit, char c);
void HT1621_WriteDigitNum(uint8_t posDigit, uint8_t data);
bool HT1621_WriteDigitChar(uint64_t posDigit, char c);
bool HT1621_SetHiBit(uint8_t posBit, bool set);
void HT1621_Backlight(bool on);

#endif /* _HT1621_LCD_H_ */
