#include "TM1640_LED.h"

#if 1
#define TM1640_CLOCK_PIN  GPIOA, 5      // SPI SCLK
#define TM1640_DATA_PIN  GPIOA, 7       // SPI MOSI
#else     // alternative pin connection to "I2C"
#define TM1640_CLOCK_PIN  GPIOB, 8      // D15 - I2C SCL
#define TM1640_DATA_PIN  GPIOB, 9       // D14 - I2C SDA
#endif

/**
 * \brief internal "frame buffer", 16 bytes = 16 columns per 8 rows
 */
static uint8_t _data[16];

/**
 * \fn bool TM1640_Init(void)
 * \brief Init HW pins + clear framebuffer
 *
 * \return Everytime OK
 */
bool TM1640_Init(void)
{
  STM_SetPinGPIO(TM1640_CLOCK_PIN, ioPortOutputPP);
  GPIOWrite(TM1640_CLOCK_PIN, 1);
  STM_SetPinGPIO(TM1640_DATA_PIN, ioPortOutputPP);
  GPIOWrite(TM1640_DATA_PIN, 1);

  for (int i = 0; i < sizeof(_data) / sizeof(_data[0]); i++)
    _data[i] = 0;

  return true;
}

#define TM1640_WAIT {}
//TODO maybe longer wait for fast rogram clokc ?? {for(int i = 0; i < 20; i++) asm("nop");}

/**
 * \fn void TM1640_WriteByte(uint8_t)
 * \brief Write single byte to TM1640, generate pseudo-I2C "by hand"
 * \remark Internally only
 *
 * \param b Value (byte) to send
 */
static void TM1640_WriteByte(uint8_t b)
{
  for (int i = 0; i < 8; i++)
  {
    GPIOWrite(TM1640_DATA_PIN, b & (1 << i));   // serial data
    TM1640_WAIT
    GPIOWrite(TM1640_CLOCK_PIN, 1);             // serial clock pulse
    TM1640_WAIT
    GPIOWrite(TM1640_CLOCK_PIN, 0);
    TM1640_WAIT
  }
}

/**
 * \fn bool TM1640_WriteData(uint8_t*, uint8_t)
 * \brief Send multiple bytes to TM1640 with START-DATA-STOP sequence
 * \remark Internally only
 *
 * \param data Pointer to bytes to send
 * \param len Number of bytes to send
 * \return In this version always OK
 */
static bool TM1640_WriteData(uint8_t *data, uint8_t len)
{
  // like I2C_start
  TM1640_WAIT
  GPIOWrite(TM1640_DATA_PIN, 0);
  TM1640_WAIT
  GPIOWrite(TM1640_CLOCK_PIN, 0);
  TM1640_WAIT

  TM1640_WriteByte(0x40);   // 01xx 0000 = data command, normal, write, auto-inc

  // like I2C_stop
  GPIOWrite(TM1640_CLOCK_PIN, 1);
  TM1640_WAIT
  GPIOWrite(TM1640_DATA_PIN, 1);
  TM1640_WAIT

  // like I2C_start again
  TM1640_WAIT
  GPIOWrite(TM1640_DATA_PIN, 0);
  TM1640_WAIT
  GPIOWrite(TM1640_CLOCK_PIN, 0);
  TM1640_WAIT

  TM1640_WriteByte(0xc0);   // 11xx 0000 = disp adr. 00

  for (int i = 0; i < len; i++)     // send all bytes
    TM1640_WriteByte(data[i]);

  //  like I2C_stop
  GPIOWrite(TM1640_CLOCK_PIN, 1);
  TM1640_WAIT
  GPIOWrite(TM1640_DATA_PIN, 1);
  TM1640_WAIT

  return true;
}

/**
 * \fn bool TM1640_DispControl(uint8_t)
 * \brief Send "control" value (byte)
 * \remark Internally only
 *
 * \param ctrl Value to send
 * \return In this version always OK
 */
static bool TM1640_DispControl(uint8_t ctrl)
{
  TM1640_WAIT
  GPIOWrite(TM1640_DATA_PIN, 0);
  TM1640_WAIT
  GPIOWrite(TM1640_CLOCK_PIN, 0);
  TM1640_WAIT

  TM1640_WriteByte(0x80 | (ctrl & 0x0f)); // 10xx Fvvv = F = 1 = ON, vvv = intensity

  GPIOWrite(TM1640_CLOCK_PIN, 1);
  TM1640_WAIT
  GPIOWrite(TM1640_DATA_PIN, 1);
  TM1640_WAIT

  return true;
}

/**
 * \fn void TM1640_DispOff(void)
 * \brief Send command "display off"
 *
 */
void TM1640_DispOff(void)
{
  TM1640_DispControl(0x00);
}

/**
 * \fn void TM1640_DispOn(void)
 * \brief Send command "display of" + full brightness
 *
 */
void TM1640_DispOn(void)
{
  TM1640_DispControl(0x0f);   // ON with full brightness
}

/**
 * \fn void TM1640_DispBright(uint8_t)
 * \brief Send command to set brightness, 0..7 - eg. 8 levels
 *
 * \param b
 */
void TM1640_DispBright(uint8_t b)
{
  TM1640_DispControl(0x08 | (b & 0x07));
}

/**
 * \fn void TM1640_Refresh(void)
 * \brief Write content of framebuffer to TM1640 controller
 *
 */
void TM1640_Refresh(void)
{
  TM1640_WriteData(_data, sizeof(_data) / sizeof(_data[0]));
}

/**
 * \fn bool TM1640_SetByte(int, uint8_t)
 * \brief Set 1 byte to framebuffer, equals column of LED matrix
 *
 * \param pos Position = pixel column
 * \param val Value to write
 * \return
 */
bool TM1640_SetByte(int pos, uint8_t val)
{
  if (pos >= (sizeof(_data) / sizeof(_data[0])))
    return false;

  _data[pos] = val;
  return true;
}

/**
 * \fn void TM1640_SetPixel(uint, uint, bool)
 * \brief Set single pixel to 1 (light on) or 0 (dark)
 * \remark Base function for pixel-graphics
 *
 * \param x X-coordinate = column
 * \param y Y-coordinate = row
 * \param on True (1) for light on
 */
bool TM1640_SetPixel(uint8_t x, uint8_t y, bool on)
{
  if ((x >= 16) || (y >= 8))
    return false;

  if (on)
    _data[x] |= 1 << y;
  else
    _data[x] &= ~(1 << y);

  return true;
}
