#include "HT1621_LCD.h"

#define HT1621_CS   GPIOB,6           // D10
#define HT1621_WR   GPIOA,5           // D13
#define HT1621_DATA GPIOA,7           // D11
#define HT1621_BKLIGHT  GPIOA, 6      // backlight

#define HT1621_DIGITS_DEFAULT 6       // nmber of digits - default value

static int _ht1621_digits_count = HT1621_DIGITS_DEFAULT;

/**
 * \fn void HT1621_OutputData(uint32_t)
 * \brief Write bit sequence to HT1621 controller
 * \remark Internally only
 *
 * \param data - Bit stream data for sending, leading 0s not sended
 */
static void HT1621_OutputData(uint32_t data)
{
  GPIOWrite(HT1621_CS, 0);

  bool validBits = false;
  for (int i = 0; i < 32; i++, data <<= 1)    // max. 32b
  {
    GPIOWrite(HT1621_WR, 0);  // inactive edge

    bool bb = (data & 0x80000000) ? 1 : 0;

    if (bb)
      validBits = true;       // detected log 1

    if (!validBits)           // skip leading 0s
      continue;

    GPIOWrite(HT1621_DATA, bb);
    __NOP();                  // short delay
    __NOP();

    GPIOWrite(HT1621_WR, 1);  // active (rising) edge
    __NOP();                  // short delay
    __NOP();
  }

  GPIOWrite(HT1621_CS, 1);    // finish = inactivate CS
}

/**
 * \fn void HT1621_WriteCmnd(uint16_t)
 * \brief Send single 8-bit command to controller
 * \remark Internally only
 *
 * \param cmd - 8-bit command
 */
static void HT1621_WriteCmnd(uint16_t cmd)
{
  cmd &= 0xff;                // isolate 8 bit command-bits
  cmd <<= 1;                  // last bit X (not used)
  cmd |= 0x04 << 9;           // 100 = command mode

  HT1621_OutputData(cmd);     // ---- 100C CCC CCCX
}

/**
 * \fn void HT1621_WriteData(uint8_t, uint8_t)
 * \brief Direct write 4-bit data (segment mask) to data register (6 bits)
 *
 * \param adr - Data register - 0-31
 * \param data - 4-bit value
 */
void HT1621_WriteData(uint8_t adr, uint8_t data)
{
  uint16_t w = data & 0x0f;   // lower 4 bits

  w |= (adr & 0x3f) << 4;     // 6 bits
  w |= 0x05 << 10;            // 101 = data mode

  HT1621_OutputData(w);
}

/**
 * \fn void HT1621_WriteDigitData(uint8_t, uint8_t)
 * \brief Write 8-value for single LCD digit
 *
 * \param posDigit - Select digit (0-5), cutted by max. digits count
 * \param data - 8-bit mask for segments
 */
void HT1621_WriteDigitData(uint8_t posDigit, uint8_t data)
{
  uint32_t w = 0x14000;     // 0001 01AA AAAA DDDD dddd

  posDigit = (posDigit * 2) % (2 * _ht1621_digits_count); // 2 addresses to one number, only 12 halfs (6 numbers]
  w |= posDigit << 8;       // max. 6 bits, cutted by previous modulo
  w |= data;                // 8b value/mask, reverted D0..D3 - see DS !!!

  HT1621_OutputData(w);
}

static uint16_t _highBitMask = 0;
/**
 * \fn bool HT1621_SetHiBit(uint8_t, bool)
 * \brief Set decimal point or battery symbol addon for num/char show
 *
 * \param posBit 0-5 for selecte position (0,1,2 = point, 3+ = battery)
 * \param set True = show, false = hide
 * \return
 */
bool HT1621_SetHiBit(uint8_t posBit, bool set)
{
  if (posBit >= _ht1621_digits_count)       // or check >= 16 ?
    return false;

  if (set)
    _highBitMask |= 1 << posBit;            // store to private variable
  else
    _highBitMask &= ~(1 << posBit);

  return true;
}

//   4
// 0   5
//   1
// 2   6
//   3
//      7
const static uint8_t _decoderNum[10] =
{
  0b01111101,
  0b01100000,
  0b00111110,
  0b01111010,
  0b01100011,
  0b01011011,
  0b01011111,
  0b01110000,
  0b01111111,
  0b01111011,
};

/**
 * \fn void HT1621_WriteDigitNum(uint8_t, uint8_t)
 * \brief View number on selected digit
 *
 * \param posDigit Digit 0-5
 * \param data Value to show - valid 0..9
 */
void HT1621_WriteDigitNum(uint8_t posDigit, uint8_t data)
{
  uint8_t val = (data >= 10) ? 0 : _decoderNum[data];   // EMPTY or converted
  if (_highBitMask & (1 << posDigit))
    val |= 0x80;

  HT1621_WriteDigitData(posDigit, val);
}

/**
 * \struct _charBytePair_struct
 * \brief Structure to store pair ASCII code - LCD segment mask
 *
 */
typedef struct _charBytePair_struct
{
  char c;
  uint8_t b;
} CharBytePair;

/**
 * Character decoding definition list
 */
const static CharBytePair _decoderChar[] =
{
  { ' ', 0b00000000 },        // space
  { '-', 0b00000010 },        // minus
  { '\0', 0b00000000 },       // end of list/table, used for searching
};

/**
 * \fn bool HT1621_WriteDigitChar(uint64_t, char)
 * \brief View character on digit position (if decoder data vailable)
 *
 * \param posDigit - Position on LCD - 0..5
 * \param c - ASCII code of character to show
 * \return True if character found in definition table
 */
bool HT1621_WriteDigitChar(uint64_t posDigit, char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    HT1621_WriteDigitNum(posDigit, c - '0');
    return true;
  }

  /* ??? zero is empty char ? or finish ?
  if (c == 0)
  {
    HT1621_WriteDigitData(posDigit, cbPtr->b);
    true;
  }
  */

  for(CharBytePair *cbPtr = (CharBytePair *)_decoderChar; cbPtr->c != 0; cbPtr++)   // anti-warn
  {
    if (c == cbPtr->c)
    {
      uint8_t val = cbPtr->b;
      if (_highBitMask & (1 << posDigit))
        val |= 0x80;

      HT1621_WriteDigitData(posDigit, val);
      return true;
    }
  }

  return false;
}

/**
 * \fn void HT1621_Backlight(bool)
 * \brief On/off backlight LED of display
 *
 * \param on - true = on, false = off
 */
void HT1621_Backlight(bool on)
{
  GPIOWrite(HT1621_BKLIGHT, on);
  //TODO use PWM to modify intensity
}

/**
 * \fn bool HT1621_Init(int)
 * \brief Init HW signals and prepare HT1621 controller settings
 *
 * \param numOfDigits - Number of digits, 1..16, typically 6
 * \return Success
 */
bool HT1621_Init(int numOfDigits)
{
  if ((numOfDigits > 0) && (numOfDigits <= 16))
    _ht1621_digits_count = numOfDigits;
  // or use default value

  STM_SetPinGPIO(HT1621_CS, ioPortOutputPP);
  GPIOWrite(HT1621_CS, 1);
  STM_SetPinGPIO(HT1621_WR, ioPortOutputPP);
  GPIOWrite(HT1621_WR, 1);
  STM_SetPinGPIO(HT1621_DATA, ioPortOutputPP);
  GPIOWrite(HT1621_DATA, 1);

  STM_SetPinGPIO(HT1621_BKLIGHT, ioPortOutputPP);
  // GPIOWrite(HT1621_BKLIGHT, 1);   // backlight ON on startup
  HT1621_Backlight(true);

  HT1621_WriteCmnd(0b00101000);   // 0010 abXc = BIAS & CCOM
                                  // 10 = 4 common, X, 0/1 = 1/3 / 1/4 bias

  HT1621_WriteCmnd(0b00000001);   // SYS EN - 0000 0001
  HT1621_WriteCmnd(0b00000011);   // LCD ON - 0000 0011

  for (int i = 0; i < _ht1621_digits_count; i++)
    HT1621_WriteDigitChar(i, '-');

  return true;                    // success
}
