#include "MAX7219_single.h"

#ifndef SPI_IS_BUSY                     // SPI helper macro
#define SPI_IS_BUSY(SPIx) (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))
#define SPI_WAIT(SPIx)            while (SPI_IS_BUSY(SPIx))
#endif

#ifndef MAX7219_SPI                     // not set from project ?
#define MAX7219_SPI SPI1                // default value
#endif

#define SPI1_CLK    GPIOA,5             // D13 - SPI1-SCK, AF05
#define SPI1_MOSI   GPIOA,7             // D11 - SPI1-MOSI, AF05

#ifndef MAX7219_CS                      // not set from project ?
#define MAX7219_CS  GPIOB,6             // default value
#endif

/**
 * \fn bool MAX7219_InitHW(void)
 * \brief Init SPI1 and PA5/PA7 pins for it, set CS pin (default PB6]
 *
 * \return True if success
 */
bool MAX7219_InitHW(void)
{
  switch ((uint32_t)MAX7219_SPI)
  {
    case (uint32_t) SPI1:
      if (!(RCC->APB2ENR & RCC_APB2ENR_SPI1EN))
      {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
      }

      SPI1->CR1 = SPI_CR1_BR_1 | SPI_CR1_BR_0;  // 011 = clk/16 - from APB2 (max. 100MHz), MAX7219 max. 10MHz ?
      SPI1->CR1 |= SPI_CR1_MSTR;
      SPI1->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM;
      SPI1->CR1 |= SPI_CR1_CPHA | SPI_CR1_CPOL; // see RM0383 pg. 555/836 (rev 1)
      SPI1->CR2 = 0;

      SPI1->CR1 |= SPI_CR1_SPE;

      STM_SetPinGPIO(SPI1_CLK, ioPortAlternatePP);
      STM_SetAFGPIO(SPI1_CLK, 5);
      STM_SetPinGPIO(SPI1_MOSI, ioPortAlternatePP);
      STM_SetAFGPIO(SPI1_MOSI, 5);
      break;
    default:
      return false;
  }

  STM_SetPinGPIO(MAX7219_CS, ioPortOutputPP);   // D10 = CS for MAX
  GPIOWrite(MAX7219_CS, 1);                     // set CS to HIGH

  return true;
}

/**
 * \fn void MAX7219_Send16b(uint16_t)
 * \brief Send 16b value to single MAX7219 via SPI
 * \remark Internally use 2 times transfer 8b value
 *
 * \param w - value to send
 */
void MAX7219_Send16b(uint16_t w)
{
  SPI_WAIT(MAX7219_SPI);
  GPIOWrite(MAX7219_CS, 0);

  MAX7219_SPI->DR = (w >> 8);
  while (!(SPI1->SR & SPI_SR_TXE))
    ;
  MAX7219_SPI->DR = w & 0xff;
  SPI_WAIT(MAX7219_SPI);
  GPIOWrite(MAX7219_CS, 1);
}

/**
 * \fn bool MAX7219_SendDataView(uint8_t, uint8_t)
 * \brief Write 8-bit value to position, rg. binary data for digit in 7-seg layout
 *
 * \param pos Digit (number position or row) 1..8
 * \param val Value to show (use generator fot 7-segment)
 * \return Success
 */
bool MAX7219_SendDataView(uint8_t pos, uint8_t val)
{
  if (pos < 1)
    pos = 1;
  if (pos > 8)
    pos = 8;

  MAX7219_Send16b((pos << 8) + val);
  return true;
}

/**
 * \brief "Number" generator for 7-segment display module
 * \remark Represents 0-9 and A-F (eg. 16 "characters" in range 0-15)
 * \remark Bit position
 *             6
 *           1   5
 *             0
 *           2   4
 *             3
 *               7
 */
const uint8_t to7seg[16] =
{               // binary value like C++ style
  0b01111110,   // 0
  0b00110000,
  0b01101101,
  0b01111001,
  0b00110011,
  0b01011011,
  0b01011111,
  0b01110000,
  0b01111111,
  0b01111011,   // 9
  0b01110111,   // A
  0b00011111,   // b
  0b01001110,   // C
  0b00111101,   // d
  0b01001111,   // E
  0b01000111    // F
};
