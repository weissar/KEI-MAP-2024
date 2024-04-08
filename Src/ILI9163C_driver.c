#include "ILI9163C_driver.h"
#include "stm_systick.h"        // required WaitMs function

/**
 * \def BB_REG
 * \brief Special defines for simple bit-oriented access
 *
 */
#ifndef BB_REG
#define BB_REG(reg, bit) (*(uint32_t *)(PERIPH_BB_BASE + ((uint32_t)(&(reg)) - PERIPH_BASE) * 32 + 4 * (bit)))
#define BB_RAM(adr, bit) (*(uint32_t *)(SRAM_BB_BASE + ((uint32_t)(adr) - SRAM_BASE) * 32 + 4 * (bit)))
#endif

/**
 * \def PIN_SPI_xxx
 * \brief  Defines for LCD control signals on MAP module
 *
 */
#define PIN_SPI_CLK   GPIOA,5     // D13
#define PIN_SPI_DOUT  GPIOA,7     // D11
#define PIN_DISP_CS   GPIOB,6     // D10
#define PIN_DISP_RES  GPIOC,7     // D9
#define PIN_DISP_DC   GPIOA,9     // D8
#define PIN_DISP_BL   GPIOA,10    // D2

#ifndef SPI_IS_BUSY
#define SPI_IS_BUSY(SPIx) (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))
#endif

/**
 * \fn void SPI_Write_Byte(uint8_t)
 * \brief Write single byte via SPI, waits for completion
 *
 * \param val Value to write - 8-bit
 */
static void SPI_Write_Byte(uint8_t val)
{
//  while(SPI_IS_BUSY(SPI1))  // wait for finish - test TXE and BSY
  ;
  SPI1->DR = val;
  while (SPI_IS_BUSY(SPI1))
    // wait for finish - test TXE and BSY
    ;
}

/**
 * \fn void LCD_WriteReg(uint8_t)
 * \brief Write "register" value = select register for next data write
 *
 * \param reg Register (address)
 */
static void LCD_WriteReg(uint8_t reg)
{
  GPIOWrite(PIN_DISP_DC, 0);
  GPIOWrite(PIN_DISP_CS, 0);
  SPI_Write_Byte(reg);
  GPIOWrite(PIN_DISP_CS, 1);
}

/**
 * \fn void LCD_WriteData_8Bit(uint8_t)
 * \brief Write 8-bit data via SPI to previous selected register
 *
 * \param d Value to write - 8-bit
 */
static void LCD_WriteData_8Bit(uint8_t d)
{
  GPIOWrite(PIN_DISP_DC, 1);
  GPIOWrite(PIN_DISP_CS, 0);
  SPI_Write_Byte(d);
  GPIOWrite(PIN_DISP_CS, 1);
}

// at this moment not used - hide Warning
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/**
 * \fn void SPI_Write_DoubleByte(uint16_t)
 * \brief Write 16-bit value, must be selected 16-bit transfers !!
 *
 * \param val Value to write - 16-bit
 */
static void SPI_Write_DoubleByte(uint16_t val)
{
  SPI1->DR = val;
  while (SPI_IS_BUSY(SPI1))
    ;
}

/**
 * \fn void LCD_WriteData_16Bit(uint16_t)
 * \brief Write 16-bit data via SPI to previous selected register
 * \remark Use as 2x 8-bit transfer, respect in SPI settings
 *
 * \param d Value to write - 16-bit
 */
static void LCD_WriteData_16Bit(uint16_t d)
{
  GPIOWrite(PIN_DISP_DC, 1);
  GPIOWrite(PIN_DISP_CS, 0);
  SPI_Write_Byte(d >> 8);
  SPI_Write_Byte(d & 0XFF);
  GPIOWrite(PIN_DISP_CS, 1);
}

/**
 * \fn void LCD_WriteData_16Bit_Repeated(uint16_t, int)
 * \brief Write count of 16-bit values (maybe fill memorz)
 * \remark Use as 2x 8-bit transfer, respect in SPI settings
 *
 * \param d Value to write - 16-bit
 * \param cnt How many times write it
 */
static void LCD_WriteData_16Bit_Repeated(uint16_t d, int cnt)
{
  GPIOWrite(PIN_DISP_DC, 1);
  GPIOWrite(PIN_DISP_CS, 0);
  for (; cnt > 0; cnt--)
  {
    SPI_Write_Byte(d >> 8);
    SPI_Write_Byte(d & 0XFF);
  }
  GPIOWrite(PIN_DISP_CS, 1);
}

#pragma GCC diagnostic pop

/**
 * \fn void LCD_WriteCommandComplete(uint8_t, uint8_t*, int)
 * \brief Complete write action - command and array of bytes
 *
 * \param cmd Command as 8-bit value
 * \param data Pointer to data array
 * \param len Length of array
 */
static void LCD_WriteCommandComplete(uint8_t cmd, uint8_t *data, int len)
{
  GPIOWrite(PIN_DISP_CS, 0);

  GPIOWrite(PIN_DISP_DC, 0);        // register/command
  SPI_Write_Byte(cmd);

  if (data && len)      // non NULL and length > 0
  {
    GPIOWrite(PIN_DISP_DC, 1);      // data

    for (; len > 0; len--)
    {
      SPI_Write_Byte(*data);
      data++;
    }
  }

  GPIOWrite(PIN_DISP_CS, 1);
}

// declaration of low-level drawing function
static void _ILI9163C_DrawPixel(int x, int y, uint32_t color);

//TODO make SPI and DMA configurable

// predefined SPI, based on physical connetion to MCU pins
static SPI_TypeDef *SPIx = SPI1;
// timer TIMx not used for autorefresh start
// predefined DMA variables and defines, based on used SPIx
static DMA_TypeDef *DMAx = DMA2;
static DMA_Stream_TypeDef *DMAxStream = DMA2_Stream3;
#define DMAx_Streamx_IRQHandler DMA2_Stream3_IRQHandler

/**
 * \fn void LCD_Init()
 * \brief Internal fn to set LCD controller configuration
 * \remark Based on one of Arduino Graphics Library
 *
 */
static void LCD_Init()
{
  //ST7735R Frame Rate available for ILI9163C too
  LCD_WriteReg(0xB1);
  LCD_WriteData_8Bit(0x01);
  LCD_WriteData_8Bit(0x2C);
  LCD_WriteData_8Bit(0x2D);

  LCD_WriteReg(0xB2);
  LCD_WriteData_8Bit(0x01);
  LCD_WriteData_8Bit(0x2C);
  LCD_WriteData_8Bit(0x2D);

  LCD_WriteReg(0xB3);
  LCD_WriteData_8Bit(0x01);
  LCD_WriteData_8Bit(0x2C);
  LCD_WriteData_8Bit(0x2D);
  LCD_WriteData_8Bit(0x01);
  LCD_WriteData_8Bit(0x2C);
  LCD_WriteData_8Bit(0x2D);

  LCD_WriteReg(0xB4); //Column inversion
  LCD_WriteData_8Bit(0x07);

  //ST7735R Power Sequence, for ILI9163C too
  LCD_WriteReg(0xC0);
  LCD_WriteData_8Bit(0xA2);
  LCD_WriteData_8Bit(0x02);
  LCD_WriteData_8Bit(0x84);
  LCD_WriteReg(0xC1);
  LCD_WriteData_8Bit(0xC5);

  LCD_WriteReg(0xC2);
  LCD_WriteData_8Bit(0x0A);
  LCD_WriteData_8Bit(0x00);

  LCD_WriteReg(0xC3);
  LCD_WriteData_8Bit(0x8A);
  LCD_WriteData_8Bit(0x2A);
  LCD_WriteReg(0xC4);
  LCD_WriteData_8Bit(0x8A);
  LCD_WriteData_8Bit(0xEE);

  LCD_WriteReg(0xC5); //VCOM
  LCD_WriteData_8Bit(0x0E);

  //ST7735R Gamma Sequence, for ILI9163C too
  LCD_WriteReg(0xe0);
  LCD_WriteData_8Bit(0x0f);
  LCD_WriteData_8Bit(0x1a);
  LCD_WriteData_8Bit(0x0f);
  LCD_WriteData_8Bit(0x18);
  LCD_WriteData_8Bit(0x2f);
  LCD_WriteData_8Bit(0x28);
  LCD_WriteData_8Bit(0x20);
  LCD_WriteData_8Bit(0x22);
  LCD_WriteData_8Bit(0x1f);
  LCD_WriteData_8Bit(0x1b);
  LCD_WriteData_8Bit(0x23);
  LCD_WriteData_8Bit(0x37);
  LCD_WriteData_8Bit(0x00);
  LCD_WriteData_8Bit(0x07);
  LCD_WriteData_8Bit(0x02);
  LCD_WriteData_8Bit(0x10);

  LCD_WriteReg(0xe1);
  LCD_WriteData_8Bit(0x0f);
  LCD_WriteData_8Bit(0x1b);
  LCD_WriteData_8Bit(0x0f);
  LCD_WriteData_8Bit(0x17);
  LCD_WriteData_8Bit(0x33);
  LCD_WriteData_8Bit(0x2c);
  LCD_WriteData_8Bit(0x29);
  LCD_WriteData_8Bit(0x2e);
  LCD_WriteData_8Bit(0x30);
  LCD_WriteData_8Bit(0x30);
  LCD_WriteData_8Bit(0x39);
  LCD_WriteData_8Bit(0x3f);
  LCD_WriteData_8Bit(0x00);
  LCD_WriteData_8Bit(0x07);
  LCD_WriteData_8Bit(0x03);
  LCD_WriteData_8Bit(0x10);

  LCD_WriteReg(0xF0); //Enable test command
  LCD_WriteData_8Bit(0x01);

  LCD_WriteReg(0xF6); //Disable ram power save mode
  LCD_WriteData_8Bit(0x00);

  LCD_WriteReg(0x3A); //65k mode
  LCD_WriteData_8Bit(0x05);
}

#define LCD_X 2     // ? offset ??
#define LCD_Y 1
#define LCD_X_MAXPIXEL  132  //LCD width maximum memory
#define LCD_Y_MAXPIXEL  162 //LCD height maximum memory

/**
 * \enum LCD_SCAN_DIR
 * \brief Available orientation
 *
 */
typedef enum
{
  L2R_U2D = 0, //The display interface is displayed , left to right, up to down
  L2R_D2U,/**< L2R_D2U */
  R2L_U2D,/**< R2L_U2D */
  R2L_D2U,/**< R2L_D2U */

  U2D_L2R,/**< U2D_L2R */
  U2D_R2L,/**< U2D_R2L */
  D2U_L2R,/**< D2U_L2R */
  D2U_R2L,/**< D2U_R2L */
} LCD_SCAN_DIR;

#define SCAN_DIR_DFT  L2R_U2D  //Default scan direction = L2R_U2D

static uint16_t LCD_Dis_Column;  //COLUMN
static uint16_t LCD_Dis_Page;  //PAGE
static LCD_SCAN_DIR LCD_Scan_Dir;
static uint16_t LCD_X_Adjust;   //LCD x actual display position calibration
static uint16_t LCD_Y_Adjust;   //LCD y actual display position calibration

/**
 * \fn void LCD_SetGramScanWay(LCD_SCAN_DIR)
 * \brief Set orientation and mem-organization
 *
 * \param Scan_dir
 */
void LCD_SetGramScanWay(LCD_SCAN_DIR Scan_dir)
{
  //Get the screen scan direction
  LCD_Scan_Dir = Scan_dir;

  //Get GRAM and LCD width and height
  if (Scan_dir == L2R_U2D || Scan_dir == L2R_D2U || Scan_dir == R2L_U2D
      || Scan_dir == R2L_D2U)
  {
    LCD_Dis_Column = LCD_HEIGHT;
    LCD_Dis_Page = LCD_WIDTH;
    LCD_X_Adjust = LCD_X;
    LCD_Y_Adjust = LCD_Y;
  }
  else
  {
    LCD_Dis_Column = LCD_WIDTH;
    LCD_Dis_Page = LCD_HEIGHT;
    LCD_X_Adjust = LCD_Y;
    LCD_Y_Adjust = LCD_X;
  }

  // Gets the scan direction of GRAM
  uint16_t MemoryAccessReg_Data = 0;  //0x36
  switch (Scan_dir)
  {
    case L2R_U2D:
      MemoryAccessReg_Data = 0X00 | 0x00;  //x Scan direction | y Scan direction
      break;
    case L2R_D2U:
      MemoryAccessReg_Data = 0x00 | 0x80;  //0xC8 | 0X10
      break;
    case R2L_U2D:  // 0X4
      MemoryAccessReg_Data = 0x40 | 0x00;
      break;
    case R2L_D2U:  // 0XC
      MemoryAccessReg_Data = 0x40 | 0x80;
      break;
    case U2D_L2R:  //0X2
      MemoryAccessReg_Data = 0X00 | 0X00 | 0x20;
      break;
    case U2D_R2L:  //0X6
      MemoryAccessReg_Data = 0x00 | 0X40 | 0x20;
      break;
    case D2U_L2R:  //0XA
      MemoryAccessReg_Data = 0x80 | 0x00 | 0x20;
      break;
    case D2U_R2L:  //0XE
      MemoryAccessReg_Data = 0x40 | 0x80 | 0x20;
      break;
  }

  // Set the read / write scan direction of the frame memory
  LCD_WriteReg(0x36); //MX, MY, RGB mode
  LCD_WriteData_8Bit(MemoryAccessReg_Data & 0xf7); //RGB color filter panel
}

/**
 * \fn bool ILI9163C_Init(void)
 * \brief Common init function - sets HW interface and controller
 * \remark Including SPI, DMA etc.
 *
 * \return Success
 */
bool ILI9163C_Init(void)        //TODO configure rotation etc.
{
  if (!(RCC->APB2ENR & RCC_APB2ENR_SPI1EN))
  {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
  }

//  SPI1->CR1 = SPI_CR1_BR_1;      // 010 = clk/8 - z APB2 (100MHz)
//  SPI1->CR1 = SPI_CR1_BR_0;      // 001 = clk/4 - z APB2 (100MHz)
  SPI1->CR1 = 0
//      | SPI_CR1_BR_2 | SPI_CR1_BR_1      // 111 = slowest !!
      | SPI_CR1_BR_0      // 001 = clk/4 - z APB2 (100MHz)
      ;

  SPI1->CR1 |= SPI_CR1_MSTR;
  SPI1->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM;
  SPI1->CR1 |= 0; // SPI_CR1_CPHA | SPI_CR1_CPOL; // viz. RM0383 pg. 555/836 (rev 1)
  SPI1->CR2 = 0;

  SPI1->CR1 |= SPI_CR1_SPE;

  STM_SetPinGPIO(PIN_SPI_CLK, ioPortAlternatePP);   // SPI1 - SCK
  STM_SetAFGPIO(PIN_SPI_CLK, 5);
  STM_SetPinGPIO(PIN_SPI_DOUT, ioPortAlternatePP);   // SPI1 - MOSI
  STM_SetAFGPIO(PIN_SPI_DOUT, 5);

  STM_SetPinGPIO(PIN_DISP_RES, ioPortOutputPP);  //
  GPIOWrite(PIN_DISP_RES, 1);
  STM_SetPinGPIO(PIN_DISP_BL, ioPortOutputPP);  // backlight, active in 1
  GPIOWrite(PIN_DISP_BL, 0);
  STM_SetPinGPIO(PIN_DISP_CS, ioPortOutputPP);  //
  GPIOWrite(PIN_DISP_CS, 1);
  STM_SetPinGPIO(PIN_DISP_DC, ioPortOutputPP);  //
  GPIOWrite(PIN_DISP_DC, 1);

  // LCD_Reset
  WaitMs(100);
  GPIOWrite(PIN_DISP_RES, 0);
  WaitMs(100);
  GPIOWrite(PIN_DISP_RES, 1);
  WaitMs(100);

  GPIOWrite(PIN_DISP_BL, 1);        // after reset

  LCD_Init();

  LCD_SetGramScanWay(L2R_U2D);
  WaitMs(200);

  //sleep out
  LCD_WriteReg(0x11);
  WaitMs(120);

  //Turn on the LCD display
  LCD_WriteReg(0x29);

  SetHiLevelDisp(LCD_WIDTH, LCD_HEIGHT, _ILI9163C_DrawPixel);

  switch((uint32_t)DMAxStream)
  {
    case (uint32_t)DMA2_Stream3:
      if (!(RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN))
      {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA2RST;
        RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA2RST;
      }

//!      NVIC_EnableIRQ(DMA2_Stream3_IRQn);
      break;
    default:
      return false;
  }

  DMAxStream->CR &= ~DMA_SxCR_EN;

  //TODO change channel bits !!
  DMAx->LIFCR |= (DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3 | DMA_LIFCR_CDMEIF3);

  //! a zde spravne provozni nastaveni DMA
  DMAxStream->CR = 0
    | DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1  // 011 = channel 3 in stream 3
    | DMA_SxCR_DIR_0  // 01 = mem to peripheral = DMA_SxM0AR to DMA_SxPAR
    | DMA_SxCR_MSIZE_0    // 01 = 16b
    | DMA_SxCR_PSIZE_0    // 01 = 16b
    | DMA_SxCR_MINC   // memory INC
    // | DMA_SxCR_TCIE   // irq "complete" fire
    ;

  DMAxStream->PAR = (uint32_t)&(SPIx->DR);      // DEST

  NVIC_EnableIRQ(DMA2_Stream3_IRQn);            //! carefully - reuired routine & enable
  return true;
}

/**
 * \fn bool ILI9163C_SetWindow(int, int, int, int)
 * \brief Set area (in pixels) to write data block
 *
 * \param x1 Top-left X
 * \param y1 Top-left Y
 * \param x2 Bottom-right X
 * \param y2 Bottom-right Y
 * \return Now every time OK
 */
bool ILI9163C_SetWindow(int x1, int y1, int x2, int y2)
{
  /*
  //set the X coordinates
  LCD_WriteReg(0x2A);
  LCD_WriteData_8Bit(0x00); //Set the horizontal starting point to the high octet
  LCD_WriteData_8Bit((x1 & 0xff) + LCD_X_Adjust); //Set the horizontal starting point to the low octet
  LCD_WriteData_8Bit(0x00);        //Set the horizontal end to the high octet
  LCD_WriteData_8Bit(((x2 - 1) & 0xff) + LCD_X_Adjust); //Set the horizontal end to the low octet

  //set the Y coordinates
  LCD_WriteReg(0x2B);
  LCD_WriteData_8Bit(0x00);
  LCD_WriteData_8Bit((y1 & 0xff) + LCD_Y_Adjust);
  LCD_WriteData_8Bit(0x00);
  LCD_WriteData_8Bit(((y2 - 1) & 0xff) + LCD_Y_Adjust);

  LCD_WriteReg(0x2C);
  */

  static uint8_t dataX[4] = { 0x00, 0x00, 0x00, 0x00 };
  static uint8_t dataY[4] = { 0x00, 0x00, 0x00, 0x00 };

  dataX[1] = (x1 & 0xff) + LCD_X_Adjust;
  dataX[3] = ((x2 - 1) & 0xff) + LCD_X_Adjust;

  LCD_WriteCommandComplete(0x2A, dataX, 4);

  dataY[1] = (y1 & 0xff) + LCD_Y_Adjust;
  dataY[3] = ((y2 - 1) & 0xff) + LCD_Y_Adjust;

  LCD_WriteCommandComplete(0x2B, dataY, 4);

  return true;
}

/**
 * \fn bool ILI9163C_FillColor(uint16_t, int)
 * \brief Fill all pixels with color (repeating)
 * \remark Prior must be selected "window"
 *
 * \param color Color as 16-bit value - RGB 5-6-5
 * \param len Nomber of pixels to fill
 * \return
 */
bool ILI9163C_FillColor(uint16_t color, int len)
{
//  LCD_WriteReg(0x2C);       // start memory write

  GPIOWrite(PIN_DISP_DC, 0);
  GPIOWrite(PIN_DISP_CS, 0);

  SPI_Write_Byte(0x2C);       // rovnou v jednom CS

  GPIOWrite(PIN_DISP_DC, 1);
  for (; len > 0; len--)
  {
    SPI_Write_Byte(color >> 8);
    SPI_Write_Byte(color & 0XFF);
  }

  GPIOWrite(PIN_DISP_CS, 1);
  return true;
}

// Framebuffer for storing pixels
static uint16_t _bufPrimary[LCD_WIDTH * LCD_HEIGHT];
// FB copy for DMA transfer, simulated double-buffering
static uint16_t _bufDouble[LCD_WIDTH * LCD_HEIGHT];

/**
 * \fn bool ILI9163C_ClearFB(uint16_t)
 * \brief Fill entire framebuffer with selected color
 *
 * \param color Color in RGB 5-6-5
 * \return Every time success
 */
bool ILI9163C_ClearFB(uint16_t color)
{
  uint32_t x = color | (color << 16);
  uint32_t *p32 = (uint32_t *)_bufPrimary;
  for(int i = 0; i < (LCD_WIDTH * LCD_HEIGHT / 2); i++)
  {
    *p32++ = x;
  }

  return true;
}

// mode = 1 - simple handy copy mem to SPIx
// mode = 2 - blocking use DMA - one iteration from primary to doublr, second step mem2spi
// mode = 3 - non-blocking with DMA-IRQ, fire manually again
#define _BLOCK_MODE   3

#if _BLOCK_MODE == 1
bool ILI9163C_FillBlock(uint16_t *pData, int len)
{
  GPIOWrite(PIN_DISP_DC, 0);
  GPIOWrite(PIN_DISP_CS, 0);

  SPI_Write_Byte(0x2C);       // rovnou v jednom CS

  SPI1->CR1 |= SPI_CR1_DFF;   // 16b tranfers

  GPIOWrite(PIN_DISP_DC, 1);
  for (; len > 0; len--)
  {
    SPI_Write_DoubleByte(*pData);
    pData++;
  }

  GPIOWrite(PIN_DISP_CS, 1);

  SPI1->CR1 &= ~SPI_CR1_DFF;   // back to 8b tranfers
  return true;
}

bool ILI9163C_InProgress(void)
{
  return false;     // when blocking, never can be
}
#elif _BLOCK_MODE == 2
bool ILI9163C_FillBlock(uint16_t *pData, int len)
{
  GPIOWrite(PIN_DISP_DC, 0);
  GPIOWrite(PIN_DISP_CS, 0);

  SPI_Write_Byte(0x2C);       // rovnou v jednom CS

  SPI1->CR1 |= SPI_CR1_DFF;   // 16b tranfers

  GPIOWrite(PIN_DISP_DC, 1);

  DMAxStream->CR &= ~(DMA_SxCR_DIR_1 | DMA_SxCR_DIR_0);   // 00 = clear
  DMAxStream->CR |= DMA_SxCR_DIR_0;   // 01 = mem to reg = DMA_SxM0AR to DMA_SxPAR
  DMAxStream->CR &= ~DMA_SxCR_PINC;   // vypnuti periph inc

  DMAxStream->PAR = (uint32_t)&(SPIx->DR);      // DEST
  DMAxStream->M0AR = (uint32_t)pData;       // SRC - start FB
  DMAxStream->NDTR = len;     //! pocita se s tim, ze to nepretece 64k

  SPIx->CR2 |= SPI_CR2_TXDMAEN;
  DMAxStream->CR |= DMA_SxCR_EN;                  // go

  while(!(DMAx->LISR & DMA_LISR_TCIF3))       //! pozor, natvrdo ten bit !!
    ;

  while (SPI_IS_BUSY(SPI1))   // a dojed ten posledni !!
    ;

  GPIOWrite(PIN_DISP_CS, 1);

  SPI1->CR1 &= ~SPI_CR1_DFF;   // back to 8b tranfers

  DMAx->LIFCR |= (DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3 | DMA_LIFCR_CDMEIF3);
  return true;
}

void ILI9163C_Refresh(void)
{
  ILI9163C_SetWindow(0, 0, LCD_WIDTH, LCD_HEIGHT);

  // tento prenos 16k x 16b trva cca 500-550us
  DMAxStream->CR &= ~(DMA_SxCR_DIR_1 | DMA_SxCR_DIR_0);   // 00 = clear
  DMAxStream->CR |= DMA_SxCR_DIR_1;  // 10 = mem to mem = DMA_SxM0AR to DMA_SxPAR
  DMAxStream->CR |= DMA_SxCR_PINC;   // zapnuti periph inc

  DMAxStream->PAR = (uint32_t)_bufPrimary;   // SRC
  DMAxStream->M0AR = (uint32_t)_bufDouble;       // DEST - start FB pro prenos
  DMAxStream->NDTR = LCD_WIDTH * LCD_HEIGHT;     //! pocita se s tim, ze to nepretece 64k

  DMAxStream->CR |= DMA_SxCR_EN;                  // go

  while(!(DMAx->LISR & DMA_LISR_TCIF3))       //! pozor, natvrdo ten bit !!
    ;

  DMAxStream->CR &= ~DMA_SxCR_EN;                  // off
  DMAx->LIFCR |= (DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3 | DMA_LIFCR_CDMEIF3);

  ILI9163C_FillBlock(_bufDouble, LCD_WIDTH * LCD_HEIGHT);
}

bool ILI9163C_InProgress(void)
{
  return false;     // when blocking, never can be
}
#elif _BLOCK_MODE >= 3
static bool _inProgress = false, _frameFinish = true;

/**
 * \fn void DMA2_Stream3_IRQHandler(void)
 * \brief Handler of DMA (SPI and dbl-buffer memcpy)
 *
 */
void DMA2_Stream3_IRQHandler(void)
{
  if (BB_REG(DMA2->LISR, DMA_LISR_TCIF3_Pos))     // is set ? e.g. finished
  {
    DMA2->LIFCR |= (DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3 | DMA_LIFCR_CDMEIF3);

    if (DMA2_Stream3->CR & DMA_SxCR_DIR_1)        // set 1x = mem-mem
    {
      ILI9163C_SetWindow(0, 0, LCD_WIDTH, LCD_HEIGHT);

      GPIOWrite(PIN_DISP_DC, 0);
      GPIOWrite(PIN_DISP_CS, 0);

      SPI_Write_Byte(0x2C);       // access in one CS

      BB_REG(SPI1->CR1, SPI_CR1_DFF_Pos) = 1;  // SPI1->CR1 |= SPI_CR1_DFF;   // 16b tranfers

      GPIOWrite(PIN_DISP_DC, 1);

      DMA2_Stream3->CR &= ~(DMA_SxCR_DIR_1 | DMA_SxCR_DIR_0);   // 00 = clear
      DMA2_Stream3->CR |= DMA_SxCR_DIR_0;   // 01 = mem to reg = DMA_SxM0AR to DMA_SxPAR
      DMA2_Stream3->CR &= ~DMA_SxCR_PINC;   // disable periph inc

      DMA2_Stream3->PAR = (uint32_t)&(SPIx->DR);      // DEST
      DMA2_Stream3->M0AR = (uint32_t)_bufDouble;      // SRC - start FB
      DMA2_Stream3->NDTR = LCD_WIDTH * LCD_HEIGHT;    // !!! must be < 64k transfers (here 16k) !!!

      BB_REG(SPI1->CR2, SPI_CR2_TXDMAEN_Pos) = 1;     // enable
      DMA2_Stream3->CR |= DMA_SxCR_EN;                // go

      _frameFinish = true;
    }
    else                          // it was mem2spi
    {
      while (SPI_IS_BUSY(SPI1))   // wait for last transfered data !!
        ;

      GPIOWrite(PIN_DISP_CS, 1);

      BB_REG(SPI1->CR2, SPI_CR2_TXDMAEN_Pos) = 0;   // disable
      BB_REG(SPI1->CR1, SPI_CR1_DFF_Pos) = 0;   // back to 8b tranfers

      _inProgress = false;
    }

    return;
  }
  else        // another reason ? not enabled !!
  {
    _inProgress = false;          // to prevent blocking
  }

  DMA2->LIFCR |= (DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3 | DMA_LIFCR_CDMEIF3);
  return;
}

/**
 * \fn bool ILI9163C_Refresh(void)
 * \brief Start updating content of framebuffer to LCD memory
 *
 * \return Success if can it start, false if transfer in progress
 */
bool ILI9163C_Refresh(void)
{
  if (_inProgress)
    return false;

  _inProgress = true;
  _frameFinish = false;

  DMA2->LIFCR |= (DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3 | DMA_LIFCR_CDMEIF3);

  // tento prenos 16k x 16b trva cca 500-550us
  DMA2_Stream3->CR &= ~(DMA_SxCR_DIR_1 | DMA_SxCR_DIR_0);   // 00 = clear
  DMA2_Stream3->CR |= DMA_SxCR_DIR_1;  // 10 = mem to mem = DMA_SxM0AR to DMA_SxPAR
  DMA2_Stream3->CR |= DMA_SxCR_PINC;   // zapnuti periph inc

  DMA2_Stream3->PAR = (uint32_t)_bufPrimary;   // SRC
  DMA2_Stream3->M0AR = (uint32_t)_bufDouble;       // DEST - start FB pro prenos
  DMA2_Stream3->NDTR = LCD_WIDTH * LCD_HEIGHT;     //! pocita se s tim, ze to nepretece 64k

  DMA2_Stream3->CR |= DMA_SxCR_TCIE;          // enable IRQ, kdyby nebylo ...
  DMA2_Stream3->CR |= DMA_SxCR_EN;            // start transfer

  return true;
}

/**
 * \fn bool ILI9163C_InProgress(void)
 * \brief Returns progress state
 *
 * \return True if transfer in progress
 */
bool ILI9163C_InProgress(void)
{
  return _inProgress;
}

/**
 * \fn bool ILI9163C_FrameFinish(void)
 * \brief Flag if frame-transfer finished
 *
 * \return Set after transfer, must be clear by main code
 */
bool ILI9163C_FrameFinish(void)
{
  return _frameFinish;
}
#else
#error Unknown BLOCK_MODE
#endif

static bool _rotate180 = false;
/**
 * \fn void ILI9163C_Rotate180(bool)
 * \brief Set rotate flag 180 deg - used in drawpixel function
 *
 * \param toRotate True for rotate
 */
void ILI9163C_Rotate180(bool toRotate)
{
  _rotate180 = toRotate;
}

/**
 * \fn void _ILI9163C_DrawPixel(int, int, uint32_t)
 * \brief Internal pixel-function - write single pixel data to framebuffer
 *
 * \param x Coordinate X
 * \param y Coordinate Y
 * \param color RGB color 5-6-5
 */
static void _ILI9163C_DrawPixel(int x, int y, uint32_t color)
{
  uint16_t c16 = (uint16_t)(color & 0xffff);  // cut 32b to 16b value
#if 1 // !USE_DMA
  c16 = __REV16(c16);       // swap bytes due byte order via SPI
#endif
  if ((x >= 0) && (x < LCD_WIDTH) && (y >= 0) && (y < LCD_HEIGHT))
  {
    if (_rotate180)
      _bufPrimary[LCD_WIDTH - 1 - x + (LCD_HEIGHT - 1 - y) * LCD_WIDTH] = c16;
    else
      _bufPrimary[x + y * LCD_WIDTH] = c16;    // cutted color
  }
}
