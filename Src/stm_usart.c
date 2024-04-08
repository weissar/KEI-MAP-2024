#include "stm_usart.h"

bool Usart2Init(int baudSpeed)
{
  STM_SetPinGPIO(GPIOA, 2, ioPortAlternatePP);
  STM_SetAFGPIO(GPIOA, 2, 7);
  STM_SetPinGPIO(GPIOA, 3, ioPortAlternatePP);
  STM_SetAFGPIO(GPIOA, 3, 7);

  if (!(RCC->APB1ENR & RCC_APB1ENR_USART2EN))       // neni povolen USART2
  {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
  }

  USART2->CR1 = USART_CR1_RE | USART_CR1_TE; // potreba povoleni prijmu a vysilani
  USART2->CR2 = 0;      // nic specialniho
  USART2->CR3 = 0;      // nic specialniho

#if 0
  //TODO doplnit vypocet BRR podle pozadovaneho a podle clocku
  USART2->BRR = 0x1A1;            // rychlost 38400 pri 16MHz - spocitano predem
#else
  // bd = apb1 / (16 x DIV) .... DIV = APB1 / (16 x BD) = 16M / (16 x baud)
  // pro baud = 38400 vyjde DIV = 26,041666, 1/16 = 0,625, tedy 26 = 0x1A a spodni 4b = 0x1
  {  // blok pro lokalni promenne
    uint32_t sampling = (USART2->CR1 & USART_CR1_OVER8) ? 8 : 16;
    uint32_t apb1, mant, tmp, frac;

    apb1 = SystemCoreClock;         //TODO zjistit presne z RCC->CFGR registru deleni

    mant = apb1 * 25 / (sampling * baudSpeed);  // ve dvacetinachnach
    tmp = mant / 25;

    frac = mant - (tmp * 25);        // zbyvajici cast 0-19 nutno prevest na 0-15
    frac = ((((frac * sampling) + 12) / 25)); // +10 kvůli zaokrouhlovani ořezem integeru

    USART2->BRR = (tmp << 4) | (frac & 0x0f); // mantisa vyssich 12b, zlomek dolni 4b, celkem 16b
  }
#endif

  USART2->CR1 |= USART_CR1_UE;                // az na zaver povolen blok USARTu

  // zrusit bufferovani vystupu i vstupu
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stdin, NULL, _IONBF, 0);

  return true;
}

int Usart2Send(char c)
{
  while (!(USART2->SR & USART_SR_TXE))
    ;           // cekej dokud neni volny TDR

  USART2->DR = c;    // zapis do TDR k odeslani
  return c;
}

int Usart2Recv(void)
{
  while (!(USART2->SR & USART_SR_RXNE))
    // cekej dokud neprijde
    ;

  return USART2->DR;      // vycti a vrat jako hodnotu
}

bool IsUsart2Recv(void)   // priznak, ze je neco v bufferu
{
  return (USART2->SR & USART_SR_RXNE) != 0;
  // podminka vynuti true/false vysledek
}

// addon for CubeIDE with new structure of syscalls.c
int __io_putchar(int ch){ return Usart2Send(ch); }
int __io_getchar(void){ return Usart2Recv(); }
// end of addon


