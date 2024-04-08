#include "HD44780_LCD.h"
#include "stm_systick.h"   // for WaitMs based on Systick

// D4 LCD DB4
// D5 LCD DB5
// D6 LCD DB6
// D7 LCD DB7
// D8 LCD Register Select, RS
// D9 LCD Enable, En
// D10  PWM control for Backlight brightness

#define HD44780_DB4 GPIOB,5
#define HD44780_DB5 GPIOB,4
#define HD44780_DB6 GPIOB,10
#define HD44780_DB7 GPIOA,8
#define HD44780_RS  GPIOA,9
#define HD44780_EN  GPIOC,7

#define LCD_BACKLIGHT GPIOB,6       // PWM4/1, I2C1SCL, UART1TX

#define PULSE_NOP_COUNT 1000

/**
 * \fn void HD44780_write_nibble(uint8_t)
 * \brief Write 4bit value to pins with E pulse
 * \remark Internal use only
 *
 * \param b One 4-bit value (higher 4 bits ignored)
 */
static void HD44780_write_nibble(uint8_t b)
{
  GPIOWrite(HD44780_DB4, b & 0x01);
  GPIOWrite(HD44780_DB5, b & 0x02);
  GPIOWrite(HD44780_DB6, b & 0x04);
  GPIOWrite(HD44780_DB7, b & 0x08);

  GPIOWrite(HD44780_EN, 1);
  for(int i = 0; i < PULSE_NOP_COUNT; i++)
    __NOP();
  GPIOWrite(HD44780_EN, 0);
}

/**
 * \fn void HD44780_write_byte(uint8_t, bool)
 * \brief Write 8-bit value to controller - Command or Data
 * \remark Internal use only
 *
 * \param b Value (8-bit) to write
 * \param cmnd Command = 0 (false), Data = 1 (true)
 */
static void HD44780_write_byte(uint8_t b, bool cmnd)
{
  GPIOWrite(HD44780_RS, !cmnd);           // command = 0, data = 1

  HD44780_write_nibble(b >> 4);           // hi-nibble
  WaitMs(5);
  HD44780_write_nibble(b & 0x0f);         // lo-nibble
  WaitMs(5);
}

/**
 * \fn bool HD44780_init_hw(void)
 * \brief Initialize signals (pins) for LCD connection
 *
 * \return Success - now every time OK
 */
bool HD44780_init_hw(void)
{
  STM_SetPinGPIO(HD44780_DB4, ioPortOutputPP);
  STM_SetPinGPIO(HD44780_DB5, ioPortOutputPP);
  STM_SetPinGPIO(HD44780_DB6, ioPortOutputPP);
  STM_SetPinGPIO(HD44780_DB7, ioPortOutputPP);

  STM_SetPinGPIO(HD44780_RS, ioPortOutputPP);
  STM_SetPinGPIO(HD44780_EN, ioPortOutputPP);
  GPIOWrite(HD44780_EN, 0);           // inactive state - for sure

  //TODO set PWM for backlight PWM

  return true;
}

/**
 * \fn bool HD44780_init_sw(void)
 * \brief Initialize LCD controller - interface, default parameters
 * \remark TODO selectable 4/8-bit interface
 *         TODO selectable pins
 *
 * \return Success - now every time OK
 */
bool HD44780_init_sw(void)
{
  GPIOWrite(HD44780_RS, 0);           // set to command mode

  HD44780_write_nibble(3);
  WaitMs(10);
  HD44780_write_nibble(3);
  WaitMs(5);
  HD44780_write_nibble(3);
  WaitMs(5);

  HD44780_write_nibble(0x02);          // 4-bit mode
  WaitMs(5);

  HD44780_write_byte(0x10, 1);        // no moves/shifts
  HD44780_write_byte(0x0F, 1);        // Display ON, Cursor ON, blink on
  HD44780_write_byte(0x01, 1);        // Clear Display
  WaitMs(10);

  HD44780_write_byte(0x80, 1);        // Move the cursor to beginning of first line
  return true;
}

/**
 * \fn void HD44780_set_cursor(uint32_t, uint32_t)
 * \brief Set cursor position
 * \remark Data-writes automatically increment cursor position
 *
 * \param col Column from 0 to predefined (Col-Count - 1)
 * \param row Row from 0 to predefined (Row-Count - 1)
 */
void HD44780_set_cursor(uint32_t col, uint32_t row)
{
  if (col >= LCD_COLUMNS_COUNT)
    col = LCD_COLUMNS_COUNT - 1;
  if (row >= LCD_ROWS_COUNT)
    row = LCD_ROWS_COUNT - 1;

  HD44780_write_byte(0x80 + 0x40 * row + col, 1);
}

/**
 * \fn void HD44780_putchar(char)
 * \brief Write single character to LCD controller
 * \remark Automatically increment cursor position
 * \remark Not checked display size-overflow !!
 *
 * \param c Character to write at cursor position
 */
void HD44780_putchar(char c)
{
  HD44780_write_byte(c, 0);
}

/**
 * \fn void HD44780_putstring(char*)
 * \brief Write string (0-terminated sequence of chars) to LCD
 * \remark Automatically increment cursor position for chars
 * \remark Not checked display-size overflow !!
 *
 * \param cp Pointer to first char of string
 */
void HD44780_putstring(char* cp)
{
  for(; *cp; cp++)
    HD44780_putchar(*cp);
}
