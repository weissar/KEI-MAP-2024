#include "stm_i2c.h"          // obsahuje i CORE apod.
#include "OLED_SH1106.h"

#define SH1106_LCDWIDTH                  128
#define SH1106_LCDHEIGHT                 64

//HACK SSH1106 has the same commands codes as SSD136

// https://github.com/adafruit/Adafruit_SSD1305_Library
#define SSD1305_SETLOWCOLUMN 0x00
#define SSD1305_SETHIGHCOLUMN 0x10
#define SSD1305_MEMORYMODE 0x20
#define SSD1305_SETCOLADDR 0x21
#define SSD1305_SETPAGEADDR 0x22
#define SSD1305_SETSTARTLINE 0x40

#define SSD1305_SETCONTRAST 0x81
#define SSD1305_SETBRIGHTNESS 0x82

#define SSD1305_SETLUT 0x91

#define SSD1305_SEGREMAP 0xA0
#define SSD1305_DISPLAYALLON_RESUME 0xA4
#define SSD1305_DISPLAYALLON 0xA5
#define SSD1305_NORMALDISPLAY 0xA6
#define SSD1305_INVERTDISPLAY 0xA7
#define SSD1305_SETMULTIPLEX 0xA8
#define SSD1305_DISPLAYDIM 0xAC
#define SSD1305_MASTERCONFIG 0xAD
#define SSD1305_DISPLAYOFF 0xAE
#define SSD1305_DISPLAYON 0xAF

#define SSD1305_SETPAGESTART 0xB0

#define SSD1305_COMSCANINC 0xC0
#define SSD1305_COMSCANDEC 0xC8
#define SSD1305_SETDISPLAYOFFSET 0xD3
#define SSD1305_SETDISPLAYCLOCKDIV 0xD5
#define SSD1305_SETAREACOLOR 0xD8
#define SSD1305_SETPRECHARGE 0xD9
#define SSD1305_SETCOMPINS 0xDA
#define SSD1305_SETVCOMLEVEL 0xDB

#define ADDR_I2C  0x78        // 0111 100x - 7bit left aligned
//#define ADDR_I2C  0x3c        // x011 1100 - right aligned

/**
 * \fn void OLED_SH1106_WriteCmd(uint8_t)
 * \brief Send command to display (via I2C)
 *
 * \param cmd Command as 8b value
 */
static void OLED_SH1106_WriteCmd(uint8_t cmd)
{
  static uint8_t buf[2] =
  {
      0x00,                             // Co = 0, C/D = 0
      0x00                              // memory place for value
  };

  buf[1] = cmd;
  I2C1_WriteBytes(ADDR_I2C, buf, 2);     // write 2 bytes
}

// functions arn't used yet
// suppress warning "'fn' defined but not used"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

static void OLED_SH1106_SetPage(int pg)
{
  uint8_t buf[4] = { 0x00, SSD1305_SETPAGEADDR, 0x00, 0x00 };  // (Co = 0, C/D = 0), cmd, pg % 4, pg % 4
  buf[2] = (pg % 8);
  buf[3] = (pg % 8);

  I2C1_WriteBytes(ADDR_I2C, buf, 4);     // write block of data
}

static void OLED_SH1106_SetAllRow(void)
{
  uint8_t buf[4] = { 0x00, SSD1305_SETCOLADDR, 0x00, 0x00 };   // (Co = 0, C/D = 0), cmd, 0, OLED_WIDTH_BYTES
  buf[2] = 0;
  buf[3] = 128;

  I2C1_WriteBytes(ADDR_I2C, buf, 4);     // write block of data
}
#pragma GCC diagnostic pop

// frame buffer, 1bit/pixel
static uint8_t _imageBuffer[SH1106_LCDWIDTH * SH1106_LCDHEIGHT / 8];

/**
 * \fn void OLED_SH1106_DrawPixel(int, int, uint32_t)
 * \brief Low-level function for single pixel, depends on framebuffer organization
 * \remark Internal (eg. static) in this module
 *
 * \param x Horizontal coordinate
 * \param y Vertical coordinate
 * \param color For single color OLED is 0 = background, !0 = foreground
 */
static void OLED_SH1106_DrawPixel(int x, int y, uint32_t color)
{
  if ((x >= 0) && (x < SH1106_LCDWIDTH)
    && (y >= 0) && (y < SH1106_LCDHEIGHT))
  {
    uint8_t *bufPtr = _imageBuffer + (y / 8) * SH1106_LCDWIDTH + x;
    if (color)                      // zero or non-zero represents monochrome "colors"
      *bufPtr |= 1 << (y & 0x07);   // maybe faster than y % 8
    else
      *bufPtr &= ~(1 << (y & 0x07));
  }
}

/**
 * \fn bool OLED_SH1106_Init(void)
 * \brief
 * \remark Use I2C1 on default pins
 *
 * \return Success - can be false if initI2C fail
 */
bool OLED_SH1106_Init(void)
{
  {   // clear framebuffer
    uint32_t *uPtr = (uint32_t *)_imageBuffer;
    for(int i = 0; i < sizeof(_imageBuffer) / 4; i++)
      *uPtr++ = 0;
  }

  //TODO variant for SPI connection
  if (!InitI2C1(i2cSpeed400k))
    return false;

  // WaitMs(50);
  for(int i = 0; i < 10000; i++)
    __asm("NOP");                      // conditionally for ARM/KEIL and GCC compilers

  OLED_SH1106_WriteCmd(SSD1305_DISPLAYOFF);                    // 0xAE
  OLED_SH1106_WriteCmd(SSD1305_SETDISPLAYCLOCKDIV);            // 0xD5
  OLED_SH1106_WriteCmd(0x80);                                  // the suggested ratio 0x80

  OLED_SH1106_WriteCmd(SSD1305_SETMULTIPLEX);                  // 0xA8
  OLED_SH1106_WriteCmd(SH1106_LCDHEIGHT - 1);                  // base on rows

  OLED_SH1106_WriteCmd(SSD1305_SETDISPLAYOFFSET);              // 0xD3
  OLED_SH1106_WriteCmd(0x0);                                   // no offset
  OLED_SH1106_WriteCmd(0);            // line #0
  OLED_SH1106_WriteCmd(0x8d);                    // 0x8D
  OLED_SH1106_WriteCmd(0x14);

  OLED_SH1106_WriteCmd(SSD1305_MEMORYMODE);                    // 0x20
  OLED_SH1106_WriteCmd(0x00);                                  // 0x0 act like ks0108, 00b, Horizontal Addressing Mode
  OLED_SH1106_WriteCmd(SSD1305_SEGREMAP | 0x1);                // 1 = column address 131 is mapped to SEG0
  OLED_SH1106_WriteCmd(SSD1305_COMSCANDEC);

  OLED_SH1106_WriteCmd(SSD1305_SETCOMPINS);                    // 0xDA
  OLED_SH1106_WriteCmd(0x12);
  OLED_SH1106_WriteCmd(SSD1305_SETCONTRAST);                   // 0x81
  OLED_SH1106_WriteCmd(0xCF);                                  // reset = 0x80

  OLED_SH1106_WriteCmd(SSD1305_SETPRECHARGE);                  // 0xd9
  OLED_SH1106_WriteCmd(0xF1);

  OLED_SH1106_WriteCmd(0xd8);                 // 0xD8
  OLED_SH1106_WriteCmd(0x40);
  OLED_SH1106_WriteCmd(SSD1305_DISPLAYALLON_RESUME);           // 0xA4
  OLED_SH1106_WriteCmd(SSD1305_NORMALDISPLAY);                 // 0xA6

//TODO  OLED_XXX_WriteCmd(SSD1306_DEACTIVATE_SCROLL);

  OLED_SH1106_WriteCmd(SSD1305_DISPLAYON);          //--turn on oled panel

  // WaitMs(50);
  for(int i = 0; i < 50000; i++)
    __asm("NOP");

  SetHiLevelDisp(SH1106_LCDWIDTH, SH1106_LCDHEIGHT, OLED_SH1106_DrawPixel);
  return true;
}

/**
 * \fn bool OLED_SH1106_UpdateContent(void)
 * \brief Update full display content from framebuffer
 * \remark Long duration - transfer ca. 1kB (128B per data-line, 8 lines)
 *         For I2C speed 100k approx. 100ms, for 400k faster - 25ms
 *         Preferred I2C speed 400k !!
 *
 * \return Success of all I2C transfers
 */
bool OLED_SH1106_UpdateContent(void)
{
  uint8_t xbuf[2] = { 0x00, 0x00 };

  bool bbResult = true;
  for(int i = 0; i < 8; i++)                // 8 pages ... view 64 pix height
  {
    xbuf[1] = SSD1305_SETPAGESTART + (i % 8);   // page 1-8
    I2C1_WriteBytes(ADDR_I2C, xbuf, 2);

    // set position to column 0
    xbuf[1] = SSD1305_SETHIGHCOLUMN + 0;
    I2C1_WriteBytes(ADDR_I2C, xbuf, 2);     // start data-column HI

    xbuf[1] = SSD1305_SETLOWCOLUMN + 2;     // practically checked !! required 2 columns shift :-(
    I2C1_WriteBytes(ADDR_I2C, xbuf, 2);     // start data-column LO

    bbResult = bbResult &&
      I2C1_WriteBytesPre(ADDR_I2C, 0x40, _imageBuffer + i * 128, 128);
  }

  return bbResult;
}

/**
 * \fn bool OLED_SH1106_DemoCntRow(uint8_t)
 * \brief Generate "counter pattern" on 8-pixel data row
 *
 * \param bRow Selected data row, in range 0-7
 * \return Success = valid row number
 */
bool OLED_SH1106_DemoCntRow(uint8_t bRow)
{
  if (bRow > 7)
    return false;

  uint8_t *ptrLine = _imageBuffer + 128 * bRow;
  for(int i = 0; i < 128; i++)
    ptrLine[i] = i;

  return true;
}
