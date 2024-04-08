#ifndef _HD44780_LCD_H
#define _HD44780_LCD_H

#include "stm_core.h"

#define LCD_COLUMNS_COUNT   16
#define LCD_ROWS_COUNT      2

bool HD44780_init_hw(void);
bool HD44780_init_sw(void);
void HD44780_set_cursor(uint32_t col, uint32_t row);
void HD44780_putchar(char c);
void HD44780_putstring(char* cp);

#endif // _HD44780_LCD_H
