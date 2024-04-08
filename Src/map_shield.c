#include "map_shield.h"

void Init8LED(void)
{
  if (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOCEN))    // neni aktivni GPIOC ?
  {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;        // povol hodiny
    RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOCRST;     // udelej RESET puls periferii
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_GPIOCRST;
  }

  GPIOC->MODER &= 0xffff0000;    // nizsich 8 I/O nastaveno na 00 kazdy = defaultni stav
  GPIOC->MODER |= 0x00005555;    // a pak jim nastav kazdemu 01 = output, bitove zapis ... 0101 0101 0101

  GPIOC->OTYPER &= 0xff00;       // kazdy vystup reprezentuje 1 bit, 0 = push-pull

  GPIOC->OSPEEDR |= 0x0000ffff;  // 2 bity na vystup, kombinace 11 = high speed
  return;
}

void Write8LED(uint8_t val)
{
  GPIOC->ODR = (GPIOC->ODR & 0xff00) | ((uint16_t)val);  // zachovat hornich 8b a prepsat spodnich 8b hodnotou
  return;
}

#define USE_SPI

void InitSPILED(void)
{
  STM_SetPinGPIO(GPIOA, 8, ioPortOutputPP); // SHIFT – LE/RCLK
  GPIOWrite(GPIOA, 8, 0);
  STM_SetPinGPIO(GPIOA, 9, ioPortOutputPP); // SHIFT - OE
  GPIOWrite(GPIOA, 9, 0);
#ifdef USE_SPI
  STM_SetPinGPIO(GPIOA, 5, ioPortAlternatePP); // SHIFT - DATA
  STM_SetAFGPIO(GPIOA, 5, 5);            // AF05 je SPI1_SCK
  STM_SetPinGPIO(GPIOA, 7, ioPortAlternatePP); // SHIFT - CLK
  STM_SetAFGPIO(GPIOA, 7, 5);            // AF05 je SPI1_MOSI

  if (!(RCC->APB2ENR & RCC_APB2ENR_SPI1EN))
  {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
  }

  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;
     // CPOL = 0, CPHA = 0, DFF = 0 = 8b data, BR = 0 = clk/2
  SPI1->CR2 = 0;              // zadne preruseni ani DMA
  SPI1->CR1 |= SPI_CR1_SPE;   // SPI enable
#else
  STM_SetPinGPIO(GPIOA, 5, ioPortOutputPP);   // SHIFT - CLK
  GPIOWrite(GPIOA, 5, 0);
  STM_SetPinGPIO(GPIOA, 7, ioPortOutputPP);   // SHIFT - DATA
#endif
}

void WriteSPILED(uint8_t val)
{
#ifdef USE_SPI
  //TODO otestuj, zda je odvysilano predchozi
  SPI1->DR = val;

  while(!(SPI1->SR & SPI_SR_TXE) || (SPI1->SR & SPI_SR_BSY))
    ;

  GPIOWrite(GPIOA, 8, 1);                   // puls na LE
  GPIOWrite(GPIOA, 8, 0);
#else
  for(int i = 0; i < 8; i++)
    {
      GPIOWrite(GPIOA, 7, (val & 0x80) != 0); // maska 1000 0000

      GPIOWrite(GPIOA, 5, 1);                 // puls na CLK
      GPIOWrite(GPIOA, 5, 0);

      val <<= 1;
    }

    GPIOWrite(GPIOA, 8, 1);                   // puls na LE
    GPIOWrite(GPIOA, 8, 0);
#endif
}

