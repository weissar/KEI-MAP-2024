#include "ST7565R_driver.h"

#include "stm_core_addon.h"       // getbusclock etc
#include "display_hilevel.h"      // connect drawpixel function to hilevel lib

#ifndef NULL                // maybe required for "invalid value"
#define NULL  ((void *)0)
#endif

/**
 * \def BB_REG, BB_RAM
 * \brief Simple access to bits via bitbanding
 *
 */
#ifndef BB_REG
#define BB_REG(reg, bit) (*(uint32_t *)(PERIPH_BB_BASE + ((uint32_t)(&(reg)) - PERIPH_BASE) * 32 + 4 * (bit)))
#define BB_RAM(adr, bit) (*(uint32_t *)(SRAM_BB_BASE + ((uint32_t)(adr) - SRAM_BASE) * 32 + 4 * (bit)))
#endif

// can be set from project symbol settings
#define USE_DMA_REFRESH

/**
 * Global settings - enable DMA, enable/set timer for auto-refresh
 */
#ifndef USE_DMA_REFRESH
#warning DMA auto refresh is not used. Do not forget to call MBED_LCD_VideoRam2LCD() after changing the frame-buffer content
#endif

#define REFRESH_TIMER 4

/**
 * Used SPI channel
 */
#define _MBED_LCD_SPI         SPI1

#define _MBED_LCD_SPI_SCK         GPIOA,5   ///< SPI clock
#define _MBED_LCD_SPI_MOSI        GPIOA,7   ///< SPI MOSI signal
#define _MBED_LCD_PIN_RSTN        GPIOA,6   ///< display RST signal, active in LO
#define _MBED_LCD_PIN_CSN         GPIOB,6     ///< display CS signal, active in LO
#define _MBED_LCD_PIN_A0          GPIOA,8   ///< display A0 signal, LO = commands, HI = data

#ifndef SPI_IS_BUSY
/**
 * Differencies between platforms
 * !! not completed, can emit error
 */
#if defined(STM32F4)
#define SPI_IS_BUSY(SPIx) (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))
//TODO #elseif another platform, for example - F103: SPIx->SR & SPI_SR_BSY
#else
#error Not supported platform
#endif
#endif // def ? SPI_IS_BUSY

/**
 * Physical dimensions of LCD, controler can support 132x64 max
 */
#define _MBED_LCD_WIDTH   128           ///< Horizontal pixels count
#define _MBED_LCD_HEIGHT  32            ///< Vertical pixels couns
#define _MBED_LCD_LINES     (_MBED_LCD_HEIGHT / 8)              ///< Count of 8x8 chars horizontaly

/**
 *  Private memory buffer
 *  Reuqired LINES * COLUMNS bytes at BSS segment
 */
static uint8_t m_videoRam[_MBED_LCD_HEIGHT / 8][_MBED_LCD_WIDTH];
static volatile bool _refreshInProgress = false;

/**
 * \fn void MBED_LCD_send(uint8_t, bool)
 * \brief
 *
 * \param val
 * \param a0
 */
static void MBED_LCD_send(uint8_t val, bool a0)       ///< Write single value to LCD - using SPI, A0 selects CMD = 0, DATA = 1
{
  GPIOWrite(_MBED_LCD_PIN_A0, a0 ? 1 : 0);
  GPIOWrite(_MBED_LCD_PIN_CSN, 0);

  _MBED_LCD_SPI->DR = val;
  while(SPI_IS_BUSY(_MBED_LCD_SPI))                   // waiting is different fo F4xx and another Fxxx
    ;                                                 // blocking waiting

  GPIOWrite(_MBED_LCD_PIN_CSN, 1);
}

#ifndef USE_DMA_REFRESH
/**
 * \fn bool MBED_LCD_reset(void)
 * \brief Send data block manually without DMA
 *
 */
static void MBED_LCD_sendData(uint8_t *val, uint16_t len)   ///< Write block of data, pointer to start and length
{
  GPIOWrite(_MBED_LCD_PIN_A0, 1);                       // always data
  GPIOWrite(_MBED_LCD_PIN_CSN, 0);

  for(; len; len--)
  {
    _MBED_LCD_SPI->DR = *val;
    val++;

    while(_MBED_LCD_SPI->SR & SPI_SR_BSY)               // blocking wait
      ;
  }

  GPIOWrite(_MBED_LCD_PIN_CSN, 1);
}
#endif

static bool MBED_LCD_reset(void)                        ///< Perform reset sequence
{
  uint16_t w, x = 0;

  GPIOWrite(_MBED_LCD_PIN_A0, 0);
  GPIOWrite(_MBED_LCD_PIN_RSTN, 0);

  for(w = 0; w < 10000; w++)
    x++;                                                // Dummy increment prevents optimalization

  GPIOWrite(_MBED_LCD_PIN_RSTN, 1);

  for(w = 0; w < 1000; w++)
    x++;

  return (x > 0);                                       // Trick to keep variable unoptimalised ...
}

static void _MBED_LCD_set_start_line(uint8_t x)          ///< Send command to LCD, info from DS
{
  MBED_LCD_send(0x10 | ((x & 0xf0) >> 4), 0);           // (2) Display start line set = Sets the display RAM display start lineaddress - lower 6 bits
  MBED_LCD_send(0x00 | (x & 0x0f), 0);                  // (2) Display start line set = Sets the display RAM display start lineaddress - lower 6 bits
}

static void _MBED_LCD_set_page(uint8_t p)                ///< Send command to LCD, info from DS
{
   MBED_LCD_send(0xB0 | (p & 0x0f), 0);                 // (3) Page address set = Sets the display RAM page address - lower 4 bits
}

static bool _MBED_LCD_init_hw()                         ///< Init SPI, GPIO, ...
{
  STM_SetPinGPIO(_MBED_LCD_PIN_RSTN, ioPortOutputPP);
  GPIOWrite(_MBED_LCD_PIN_RSTN, 1);
  STM_SetPinGPIO(_MBED_LCD_PIN_CSN, ioPortOutputPP);
  GPIOWrite(_MBED_LCD_PIN_CSN, 1);
  STM_SetPinGPIO(_MBED_LCD_PIN_A0, ioPortOutputPP);

  STM_SetPinGPIO(_MBED_LCD_SPI_MOSI, ioPortAlternatePP);
  STM_SetAFGPIO(_MBED_LCD_SPI_MOSI, 5);        // AFxx
  STM_SetPinGPIO(_MBED_LCD_SPI_SCK, ioPortAlternatePP);
  STM_SetAFGPIO(_MBED_LCD_SPI_SCK, 5);         // AFxx

  switch((uint32_t)_MBED_LCD_SPI)                       // Switch reuired due using ohter APBx for some SPIx
  {
    case (uint32_t)SPI1:
      if (!(RCC->APB2ENR & RCC_APB2ENR_SPI1EN))
      {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
      }
      break;
    //TODO other SPIx peripheral
    default:
      return false;
  }

  _MBED_LCD_SPI->CR1 = 0
      | SPI_CR1_CPHA | SPI_CR1_CPOL     // polarity from DS
      | SPI_CR1_SSI | SPI_CR1_SSM       // required for correct function
      | SPI_CR1_MSTR;
  _MBED_LCD_SPI->CR2 = 0;

  {                                     // from DS - max clock 10MHz (100ns period)
//    uint32_t apb2 = SystemCoreClock;    //TODO calculate from RCC
    uint32_t apb2 = STM_GetBusClock(busClockAPB2);
    uint32_t BRDiv = 0;                 // 000 = pclk / 2

    if (apb2 > 20e6) BRDiv = 0x01;      // 001 = pclk / 4
    if (apb2 > 40e6) BRDiv = 0x02;      // 010 = pclk / 8
    if (apb2 > 80e6) BRDiv = 0x03;      // 011 = pclk / 16

    _MBED_LCD_SPI->CR1 &= ~SPI_CR1_BR;
    //  only for F4xx:
    _MBED_LCD_SPI->CR1 |= (BRDiv & 0x07) << 3;    // isolate 3 bits and set to bits 5..3
    //TODO another platforms
  }

  _MBED_LCD_SPI->CR1 |= SPI_CR1_SPE;             // enable

  return true;
}

static bool _MBED_LCD_init_hw_refresh(void)   // call after LCD init
{
#ifdef USE_DMA_REFRESH
//  bbUseDMA = false;
  if (!(RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN))
  {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA2RST;
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA2RST;
  }

  NVIC_EnableIRQ(DMA2_Stream3_IRQn);

#ifdef REFRESH_TIMER
  uint32_t apb = STM_GetTimerClock(REFRESH_TIMER);
  if (apb != 0)     // found valid Timer ?
  {
    TIM_TypeDef *timPtr = NULL;
    IRQn_Type irqN = 0;
    switch(REFRESH_TIMER)
    {
      case 4:
        if (!(RCC->APB1ENR & RCC_APB1ENR_TIM4EN))
        {
          RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
          RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST;
          RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM4RST;
        }

        timPtr = TIM4;
        irqN = TIM4_IRQn;
        break;
        //TODO maji jine citace jine registry ?
      default:
        return false;
    }

    timPtr->CR1 = TIM_CR1_URS;
    timPtr->CR2 = 0;
    //    TIM3->EGR = TIM_EGR_UG;

    timPtr->PSC = apb / 100000 - 1;   // 100us = 10kHz
    timPtr->ARR = 500 - 1;            // reload 5ms (500 x 0.01ms)

    if (irqN > 0)
    {
      timPtr->DIER |= TIM_DIER_UIE;
      NVIC_EnableIRQ(irqN);
    }

    timPtr->CR1 |= TIM_CR1_CEN;
  }
#endif
  //  bbUseDMA = true;
#endif

  // zbytecne, je to vychozi hodnota ... _refreshInProgress = false;
  return true;
}

/**
 * \fn void MBED_LCD_InitVideoRam(uint8_t)
 * \brief
 *
 * \param val
 */
static void MBED_LCD_InitVideoRam(uint8_t val)
{
  for(int r = 0; r < _MBED_LCD_HEIGHT / 8; r++)      // repaired 2019-09-23
    for(int x = 0; x < _MBED_LCD_WIDTH; x++)
      m_videoRam[r][x] = val;
}

static bool _MBED_LCD_DrawPixel(int x, int y, bool black)
{
  if ((x >= 0) && (x < _MBED_LCD_WIDTH)
    && (y >= 0) && (y < _MBED_LCD_HEIGHT))
  {
    if (black)
      m_videoRam[y / 8][x] |= 1 << (y % 8);
    else
      m_videoRam[y / 8][x] &= ~(1 << (y % 8));

    return true;
  }
  else
    return false;
}

/**
 * \fn bool MBED_LCD_init(void)
 * \brief Initialisation - HW parts and init commands for LCD controller (see DS and MBED sample init code)
 *
 * \return False if ini fails
 */
bool MBED_LCD_init(void)
{
  if (!_MBED_LCD_init_hw())  // check success of HW init
    return false;

  MBED_LCD_reset();

  MBED_LCD_send(0xAE, 0);   //  display off
  MBED_LCD_send(0xA2, 0);   //  bias voltage

  MBED_LCD_send(0xA0, 0);
  MBED_LCD_send(0xC8, 0);   //  colum normal

  MBED_LCD_send(0x22, 0);   //  voltage resistor ratio
  MBED_LCD_send(0x2F, 0);   //  power on
  //wr_cmd(0xA4);   //  LCD display ram
  MBED_LCD_send(0x40, 0);   // start line = 0
  MBED_LCD_send(0xAF, 0);   // display ON

  MBED_LCD_send(0x81, 0);   //  set contrast
  MBED_LCD_send(0x17, 0);   //  set contrast

  MBED_LCD_send(0xA6, 0);   // display normal
//  MBED_LCD_send(0xA7, 0);     // display inverted

//  MBED_LCD_send(0xa5, 0);

  _MBED_LCD_init_hw_refresh();

  MBED_LCD_InitVideoRam(0x00);      // fill content with 0 = clear memory buffer

  SetHiLevelDisp(_MBED_LCD_WIDTH, _MBED_LCD_HEIGHT, _MBED_LCD_DrawPixel);

  return true;                // ALL init OK
}

#ifdef USE_DMA_REFRESH
static volatile int _refreshDMAStage = -1;
uint8_t m_sendBuffer[_MBED_LCD_LINES * _MBED_LCD_WIDTH];
#endif

/**
 * \fn bool MBED_LCD_VideoRam2LCD(void)
 * \brief Copying content of videoRAM to LCD controller, based on SPI bulk transfer
 * \remark Duration ca 1.8ms without DMA when clock HSI 16MHz
 *         700us with DMA ?
 *
 * \return
 */
bool MBED_LCD_VideoRam2LCD(void)
{
  if (_refreshInProgress)
    return false;

  _refreshInProgress = true;
#ifdef USE_DMA_REFRESH
  DMA2_Stream3->CR &= ~DMA_SxCR_EN;

  // Writing 1 to these bits clears the corresponding flags in the DMA_LISR register
  DMA2->LIFCR = (DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3 | DMA_LIFCR_CDMEIF3);

  DMA2_Stream3->CR = 0
     | DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1  // 011 = channel 3 in stream 3
     | DMA_SxCR_DIR_1  // 10 = mem to mem = DMA_SxPAR to DMA_SxM0AR
     | DMA_SxCR_MINC
     | DMA_SxCR_PINC
     | DMA_SxCR_TCIE   // irq "complete" fire          DMA_CCR3_MINC
     ;

  DMA2_Stream3->PAR = (uint32_t)m_videoRam;             // SRC  ... ! 2-dim array
  DMA2_Stream3->M0AR = (uint32_t)m_sendBuffer;          // DEST ... ! linear array, same size

  DMA2_Stream3->NDTR = _MBED_LCD_LINES * _MBED_LCD_WIDTH;

  _refreshDMAStage = -1;                                 // start stage: MEM
  DMA2_Stream3->CR |= DMA_SxCR_EN;                      // go copying
#else
  for (uint8_t r = 0; r < _MBED_LCD_LINES; r++)
  {
    MBED_LCD_set_page(r);
    MBED_LCD_set_start_line(0);

#if 1
    MBED_LCD_sendData(m_videoRam[r], _MBED_LCD_COLUMNS); // block operation
#else
    for(uint8_t x = 0; x < _MBED_LCD_COLUMNS; x++)
      MBED_LCD_send(m_videoRam[r * _MBED_LCD_COLUMNS + x], 1);
#endif
  }

  _refreshInProgress = false;
#endif

  return true;
}

#ifdef USE_DMA_REFRESH

// HARD CODED LCD A0, CS pins !!!
void DMA2_Stream3_IRQHandler(void)
{
  if (DMA2->LISR & DMA_LISR_TCIF3)
  {
    DMA2->LIFCR = DMA_LIFCR_CTCIF3;   // only write 1 available

    if (_refreshDMAStage >= 0)        // not for M2M transfer, which is -1
    {
      while(_MBED_LCD_SPI->SR & SPI_SR_BSY)    // while sending is not finished
        ;                                      // (see. Figure 205. Transmission using DMA - pg.572/836 RM F411)

      BB_REG(GPIOB->ODR, 6) = 1;      // to inactive CS
    }

    // everytime is needed to clear all including errors, sometimes was set FEIFx ??
    DMA2->LIFCR = (DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3 | DMA_LIFCR_CDMEIF3);

    _refreshDMAStage++;
    if (_refreshDMAStage >= _MBED_LCD_LINES)
    {
      DMA2_Stream3->CR &= ~(DMA_SxCR_EN | DMA_SxCR_TCIE);   // stop and disable irq

      _MBED_LCD_SPI->CR2 &= ~SPI_CR2_TXDMAEN;
      _refreshInProgress = false;
    }
    else
    {
      DMA2_Stream3->CR &= ~DMA_SxCR_EN;

      _MBED_LCD_SPI->CR2 &= ~SPI_CR2_TXDMAEN;

      _MBED_LCD_set_page(_refreshDMAStage);
      _MBED_LCD_set_start_line(0);

      DMA2_Stream3->CR = 0
        | DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1  // 011 = channel 3 in stream 3
        | DMA_SxCR_DIR_0  // 01 = mem to peripheral = DMA_SxM0AR to DMA_SxPAR
        | DMA_SxCR_MINC
        | DMA_SxCR_TCIE   // irq "complete" fire          DMA_CCR3_MINC
        ;

      DMA2_Stream3->PAR = (uint32_t)&(SPI1->DR);      // DEST
      DMA2_Stream3->M0AR = (uint32_t)&(m_sendBuffer[_refreshDMAStage * _MBED_LCD_WIDTH]);// SRC

      DMA2_Stream3->NDTR = _MBED_LCD_WIDTH;

      _MBED_LCD_SPI->CR2 |= SPI_CR2_TXDMAEN;

      BB_REG(GPIOA->ODR, 8) = 1;      // data transfer
      BB_REG(GPIOB->ODR, 6) = 0;      // to active CS

      DMA2_Stream3->CR |= DMA_SxCR_EN;        // go
    }
  }
}
#endif

#ifdef REFRESH_TIMER
#if (REFRESH_TIMER == 4)
void TIM4_IRQHandler(void)
{
  TIM4->SR = ~TIM_SR_UIF;     // see RM 15.4.5
  MBED_LCD_VideoRam2LCD();    // fire refresh LCD via DMA or manual cycle
}
//#elif
#else
//TODO  reagovat na to, ze to chybi
#error Invalid REFRESH_TIMER settings
#endif
#endif
