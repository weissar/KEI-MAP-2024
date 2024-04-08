#include "MAX7219_double.h"

/**
 * \def SPI_IS_BUSY
 * \brief Macro to simplify SPI operations
 *
 */
#ifndef SPI_IS_BUSY                     // SPI helper macro
#define SPI_IS_BUSY(SPIx) (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))
#define SPI_WAIT(SPIx)            while (SPI_IS_BUSY(SPIx))
#endif

#ifndef MAX7219_SPI                     // not set from project ?
#define MAX7219_SPI SPI1                // default value
#endif

//TODO change for another pins
#define SPI1_CLK    GPIOA,5             // D13 - SPI1-SCK, AF05
#define SPI1_MOSI   GPIOA,7             // D11 - SPI1-MOSI, AF05

#ifndef MAX7219_CS                      // not set from project ?
#define MAX7219_CS  GPIOB,6             // default value
#endif

/**
 * \def USE_SPI_16B
 * \brief Enable 16-bit operation on SPI
 *
 */
#define USE_SPI_16B

/**
 * \fn bool MAX7219_InitHW(void)
 * \brief Init SPI1 and PA5/PA7 pins for it, set CS pin (default PB6]
 *
 * \return True if success
 */
bool MAX7219_InitHW(void)
{
  switch ((uint32_t) MAX7219_SPI)
  {
    case (uint32_t) SPI1:
      if (!(RCC->APB2ENR & RCC_APB2ENR_SPI1EN))
      {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
      }

      STM_SetPinGPIO(SPI1_CLK, ioPortAlternatePP);
      STM_SetAFGPIO(SPI1_CLK, 5);
      STM_SetPinGPIO(SPI1_MOSI, ioPortAlternatePP);
      STM_SetAFGPIO(SPI1_MOSI, 5);
      break;
    default:
      return false;
  }

  MAX7219_SPI->CR1 = SPI_CR1_BR_1 | SPI_CR1_BR_0;  // 011 = clk/16 - from APB2 (max. 100MHz), MAX7219 max. 10MHz ?
  MAX7219_SPI->CR1 |= SPI_CR1_MSTR;
#ifdef USE_SPI_16B
  MAX7219_SPI->CR1 |= SPI_CR1_DFF;
#endif
  MAX7219_SPI->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM;
  MAX7219_SPI->CR1 |= SPI_CR1_CPHA | SPI_CR1_CPOL; // see RM0383 pg. 555/836 (rev 1)
  MAX7219_SPI->CR2 = 0;

  MAX7219_SPI->CR1 |= SPI_CR1_SPE;

  STM_SetPinGPIO(MAX7219_CS, ioPortOutputPP);   // D10 = CS for MAX
  GPIOWrite(MAX7219_CS, 1);                     // set CS to HIGH

  return true;
}

/**
 * \fn void MAX7219_Dbl_Send16b(uint16_t, int)
 * \brief Send 16-bit word to "all" modules
 * \remark Internally use 2 times transfer 8b value
 * \remark Practically only 2 MAXs
 *
 * \param w Value to send
 * \param moduleCount Number of MAXs connected in daisy-chain
 */
static void MAX7219_Dbl_Send16b(uint16_t w, int moduleCount)
{
  SPI_WAIT(MAX7219_SPI);
  GPIOWrite(MAX7219_CS, 0);

  for (; moduleCount; moduleCount--)    // simple counter
  {
#ifdef USE_SPI_16B
    SPI1->DR = w;           // full 16b value
    while (!(MAX7219_SPI->SR & SPI_SR_TXE))
      ;
#else
    MAX7219_SPI->DR = (w >> 8);
    while (!(MAX7219_SPI->SR & SPI_SR_TXE))
      ;                     //! not needed wait to complete, enough is "transmit empty"
    MAX7219_SPI->DR = w & 0xff;
    while (!(MAX7219_SPI->SR & SPI_SR_TXE))
      ;
#endif
  }

  SPI_WAIT(MAX7219_SPI);        // wait for sending completed

  GPIOWrite(MAX7219_CS, 1);
}

/**
 * \brief "Frame buffer" for 16 columns 8 pix-rows
 * \remark Internally use 2 times transfer 8b value
 */
static uint8_t _fbuffer[16];    //TODO make configurable count of MAXs

/**
 * \fn void MAX7219_Dbl_Refresh(void)
 * \brief Update content of framebuffer to dual MAXs
 * \remark TODO modify for selected count of MAXs
 *
 */
void MAX7219_Dbl_Refresh(void)
{
  SPI_WAIT(MAX7219_SPI);

  for (int i = 0; i < 8; i++)
  {
    GPIOWrite(MAX7219_CS, 0);

    uint16_t
    w = ((i + 1) << 8) | _fbuffer[i];
    // first send data for second MAX

#ifdef USE_SPI_16B
    SPI1->DR = w;           // full 16b value
    while (!(MAX7219_SPI->SR & SPI_SR_TXE))
      ;
#else
    MAX7219_SPI->DR = (w >> 8);
    while (!(MAX7219_SPI->SR & SPI_SR_TXE))
      ;                     //! not needed wait to complete, enough is "transmit empty"
    MAX7219_SPI->DR = w & 0xff;
    while (!(MAX7219_SPI->SR & SPI_SR_TXE))
      ;
#endif

    w = ((i + 1) << 8) | _fbuffer[i + 8];
    // now send data for first MAX

#ifdef USE_SPI_16B
    SPI1->DR = w;           // full 16b value
#else
    MAX7219_SPI->DR = (w >> 8);
    while (!(MAX7219_SPI->SR & SPI_SR_TXE))
      ;                     //! not needed wait to complete, enough is "transmit empty"
    MAX7219_SPI->DR = w & 0xff;
#endif
    SPI_WAIT(MAX7219_SPI);    // wait for sending completed

    GPIOWrite(MAX7219_CS, 1);
  }
}

/**
 * \fn void MAX7219_Dbl_Clear(void)
 * \brief Clear framebuffer (only data, not MAXs content)
 *
 */
void MAX7219_Dbl_Clear(void)
{
  for (int i = 0; i < 16; i++)
    _fbuffer[i] = 0;
}

/**
 * \fn bool MAX7219_Dbl_PutPixel(int, int, uint32_t)
 * \brief Simple function to set pixel black (0) or light (!=0)
 *
 * \param x Coordinate X, range 0-15
 * \param y Coordinate Y, range 0-7
 * \param color 0 = black, otherwise "light"
 * \return Success if valid x/y coordinates
 */
bool MAX7219_Dbl_PutPixel(int x, int y, uint32_t color)   // color 0 / non-0
{
  if ((x >= 16) || (y >= 8) || (x < 0) || (y < 0))
    return false;

  uint8_t *pb = _fbuffer + x;  // calculate column in data
  uint8_t mask = 1 << y;       // calculate pixel position
  if (color)
    *pb |= mask;
  else
    *pb &= ~mask;

  return true;
}

/**
 * \fn bool MAX7219_Dbl_InitSW(void)
 * \brief Common initialization of both MAXs
 *
 * \return Everytime OK
 */
bool MAX7219_Dbl_InitSW(void)
{
  // X0xx = NoOP
  // X1xx - X9xx = Digit 0 .. 7, MSB - DP A B C D E F G
  MAX7219_Dbl_Send16b(0x0f00, 2);    // XFxx = display test, xx00 = normal, xx01 = test
  MAX7219_Dbl_Send16b(0x0b07, 2);    // XBxx = scan limit, xxXb = 0 = digit 0 .. 7 = digit 0,1,2..7
  MAX7219_Dbl_Send16b(0x0900, 2);    // X9xx = decode-mode, xx00 = no decode
  MAX7219_Dbl_Send16b(0x0c01, 2);    // XCxx = shutdown register, xx01 = normal operation
  MAX7219_Dbl_Send16b(0x0a01, 2);    // XAxx = intensity, xxXa = a = 1 .. 31 / 32 (step 2)

  MAX7219_Dbl_Clear();

//  MAX7219_M_Send16b(0x0f00, 2);    // XFxx = display test, xx00 = normal, xx01 = test
  return true;
}

/**
 * \brief Simple font generator, 3x5 pixels, only numbers 0-9
 * \remark Internally use only
 */
static const uint8_t font_data_3x5[] =
{
    // 0x30 = 0
    0b00000010,     // .*.
    0b00000101,     // *.*
    0b00000101,     // *.*
    0b00000101,     // *.*
    0b00000010,     // .*.
    // 0x31 = 1
    0b00000010,     // .*.
    0b00000110,     // **.
    0b00000010,     // .*.
    0b00000010,     // .*.
    0b00000111,     // ***
    // 0x32 = 2
    0b00000110,     // **.
    0b00000001,     // ..*
    0b00000010,     // .*.
    0b00000100,     // *..
    0b00000111,     // ***
    // 0x33 = 3
    0b00000110,     // **.
    0b00000001,     // ..*
    0b00000010,     // .*.
    0b00000001,     // ..*
    0b00000110,     // **.
    // 0x34 = 4
    0b00000100,     // *..
    0b00000100,     // *..
    0b00000111,     // ***
    0b00000010,     // .*.
    0b00000010,     // .*.
    // 0x35 = 5
    0b00000111,     // ***
    0b00000100,     // *..
    0b00000010,     // .*.
    0b00000001,     // ..*
    0b00000110,     // **.
    // 0x36 = 6
    0b00000010,     // .*.
    0b00000100,     // *..
    0b00000110,     // **.
    0b00000101,     // *.*
    0b00000010,     // .*.
    // 0x37 = 7
    0b00000111,     // ***
    0b00000001,     // ..*
    0b00000001,     // ..*
    0b00000010,     // .*.
    0b00000010,     // .*.
    // 0x38 = 8
    0b00000010,     // .*.
    0b00000101,     // *.*
    0b00000111,     // ***
    0b00000101,     // *.*
    0b00000010,     // .*.
    // 0x39 = 9
    0b00000010,     // .*.
    0b00000101,     // *.*
    0b00000011,     // .**
    0b00000001,     // ..*
    0b00000010,     // .*.

    // 0x20 = space
    0b00000000,     // ...
    0b00000000,     // ...
    0b00000000,     // ...
    0b00000000,     // ...
    0b00000000,     // ...
};

/**
 * \fn bool MAX7219_Dbl_DrawNum_3x5(int, int, char)
 * \brief Draw character to framebuffer with 3x5 font
 *
 * \param x X-coordinate in pixels
 * \param y Y-coordinate in pixels
 * \param c Character, available '0'-'9'
 * \return Success = valid character
 */
bool MAX7219_Dbl_DrawNum_3x5(int x, int y, char c)
{
  if ((c < '0') || (c > '9'))
    return false;           // only numbers

  uint8_t *bptr = (uint8_t *)font_data_3x5 + 5 * (c - '0');
  for(int r = 0; r < 5; r++)
  {
    for (int i = 0; i < 3; i++)
    {
      MAX7219_Dbl_PutPixel(x + 2 - i, y + r,
          *bptr & (1 << i));
    }

    bptr++;
  }

  return true;
}
