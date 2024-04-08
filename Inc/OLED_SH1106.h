#ifndef _OLED_SH11106_H
#define _OLED_SH11106_H

#include "display_hilevel.h"

bool OLED_SH1106_Init(void);
bool OLED_SH1106_UpdateContent(void);

// not implemented, use DISP_Fill ... bool OLED_SH1106_Fill(uint8_t bFill);
bool OLED_SH1106_DemoCntRow(uint8_t bRow);

#endif  // _OLED_SH11106_H
