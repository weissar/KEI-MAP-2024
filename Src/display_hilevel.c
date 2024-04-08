#include "display_hilevel.h"

static void (* my_DrawPixel)(int x, int y, uint32_t barva) = NULL;

static int pixelsWidth = 0, pixelsHeight = 0;
static int textX = 0, textY = 0;

void SetHiLevelDisp(int pixWidth, int pixHeight, void *ptrDrawPixel)
{
  my_DrawPixel = ptrDrawPixel;

  pixelsWidth = pixWidth;
  pixelsHeight = pixHeight;
}

inline int DISP_GetWidth(void)
{
  return pixelsWidth;
}

inline int DISP_GetHeight(void)
{
  return pixelsHeight;
}

#include "font_atari_8x8.h"
#include "font_lcd_5x7.h"
//TODO enable other fonts from user-config
/*
#include "font_thin_8x8.h"
#include "font_system_5x7.h"
#include "font_wendy_3x5.h"
#include "font_newbasic_3x6.h"
*/

static uint8_t *bpFontBase = NULL;
static FONT_Header_t *ptrFontHeader = NULL;
static bool bbPixSpace = false;
static bool bbLineSpace = false;
static int iFontMultiply = 1;

bool DISP_SetFont(eFontsAvailable fnt)
{
  bbPixSpace = false;
  bbLineSpace = false;

  switch(fnt)
  {
    case font_atari_8x8:
    default:
      ptrFontHeader = (FONT_Header_t *)font_atari_8x8_data;

      //TODO select start of char defines, fefaultne ZA hlavickou
      bpFontBase = (uint8_t *)font_atari_8x8_data + sizeof(FONT_Header_t);
      //! pretypovani kvuli warningu "... discards const ..."
      break;
    case font_lcd_5x7:
      ptrFontHeader = (FONT_Header_t *)font_lcd_5x7_data;
      bpFontBase = (uint8_t *)font_lcd_5x7_data + sizeof(FONT_Header_t);
      bbPixSpace = true;    // takze bude realne 6x7 ...
      break;
      /*
    case font_thin_8x8:
      ptrFontHeader = (FONT_Header_t *)font_thin_8x8_data;
      bpFontBase = font_thin_8x8_data + sizeof(FONT_Header_t);
      break;
    case font_wendy_3x5:
      ptrFontHeader = (FONT_Header_t *)font_wendy_3x5_data;
      bpFontBase = font_wendy_3x5_data + sizeof(FONT_Header_t);
      break;
    case font_newbasic_3x6:
      ptrFontHeader = (FONT_Header_t *)font_newbasic_3x6_data;
      bpFontBase = font_newbasic_3x6_data + sizeof(FONT_Header_t);
      break;
      */
  }

  return true;
}

bool DISP_SetFontMultiply(int m)
{
  switch(m)
  {
    case 1:
    case 2:
    case 4:
    case 8:
      iFontMultiply = m;
      return true;
    default:
      return false;
  }
}

__inline int DISP_GetFontWidth(void)
{
  return iFontMultiply * (ptrFontHeader->width + (bbPixSpace ? 1 : 0));   // add 1 pixel space
}

__inline int DISP_GetFontHeight(void)
{
  return ptrFontHeader->height + (bbLineSpace ? 1 : 0);
}

__inline int DISP_GetCharPerRow(void)
{
  return pixelsWidth / DISP_GetFontWidth();
}

__inline int DISP_GetRowPerDisp(void)
{
  return pixelsHeight / DISP_GetFontHeight();
}

static uint32_t _textForeColor = ~0;                    // white on color panels
static uint32_t _textBackColor = 0;                     // black on color panels

void DISP_TextForeColor(uint32_t u)
{
  _textForeColor = u;
}

void DISP_TextBackColor(uint32_t u)
{
  _textBackColor = u;
}

void DISP_GotoXY(int colPix, int rowPix)
{
  textX = colPix;
  textY = rowPix;
}

void DISP_WriteChar(char c)
{
  DISP_WriteCharXY(textX, textY, c);

  textX += iFontMultiply * (ptrFontHeader->width + (bbPixSpace ? 1 : 0));
  if (textX >= pixelsWidth)
  {
    textX = 0;

    textY += iFontMultiply * (ptrFontHeader->height + (bbLineSpace ? 1 : 0));
    if (textY >= pixelsHeight)
      textY = 0;
  }
}

void DISP_WriteString(char *cp)
{
  while(*cp)
  {
    DISP_WriteChar(*cp);
    cp++;
  }
}

//! X a Y are in pixels !!
void DISP_WriteCharXY(int x, int y, char c)
{
  int i, j, ii, jj;
  uint8_t *bp;

  if (ptrFontHeader == NULL)
    DISP_SetFont(-1);     // use default font

  if ((c >= (ptrFontHeader->first + ptrFontHeader->count)) || (c < ptrFontHeader->first))
    return;

  c -= ptrFontHeader->first;
  bp = bpFontBase + (c * ptrFontHeader->width);

#if 0       // bez multiply a jen pro 8x8 fonty
  for (i = 0; i < ptrFontHeader->width; i++)
  {
    uint8_t b = bp[i];

    if (ptrFontHeader->height <= 8)
    {
      for (j = 0; j < ptrFontHeader->height; j++)
      {
        my_DrawPixel(x + i, y + j,
            (b & 1) ? _textForeColor : _textBackColor);    // LSB first
        b >>= 1;
      }

      if (bbLineSpace)
        my_DrawPixel(x + i, y + j, _textBackColor); // j je pix na radku "pod"

      continue;
    }

    /* TODO
    if (ptrFontHeader->height <= 16)
    {
      for (j = 0; j < 8; j++)
      {
        my_DrawPixel(x * 8 + i, y + j, (b & 1));    // LSB first
        b >>= 1;
      }

      continue;
    }
    */
  }

  if (bbPixSpace)
  {
    if (ptrFontHeader->height <= 8)
    {
      int offsetX = x + ptrFontHeader->width + (bbPixSpace ? 1 : 0);

      for (j = 0; j < ptrFontHeader->height; j++)
        my_DrawPixel(offsetX + i, y + j, _textBackColor);

      if (bbLineSpace)
        my_DrawPixel(offsetX + i, y + j, _textBackColor);          // j je pix na radku "pod"
    }
  }
#else
  if (ptrFontHeader->height > 8)
    return;                     // zatim neumime

  int xr = x, yr = y;
  uint8_t  b;
  for (i = 0; i < ptrFontHeader->width; i++)
  {
    for(ii = 0; ii < iFontMultiply; ii++)
    {
      b = bp[i];          // bajt dle generatoru pro 8x8 fonty
      yr = y;

      for (j = 0; j < ptrFontHeader->height; j++)
      {
        for (jj = 0; jj < iFontMultiply; jj++)
        {
          my_DrawPixel(xr, yr, (b & 1) ? _textForeColor : _textBackColor);    // LSB first
          yr++;
        }

        b >>= 1;
      }

      if (bbLineSpace)
      {
        for (jj = 0; jj < iFontMultiply; jj++)
        {
          my_DrawPixel(xr, yr, _textBackColor); // j je pix na radku "pod"
          yr++;
        }
      }

      xr++;
    }
  }

  if (bbPixSpace)
  {
    // zustava posledni XR
    for (ii = 0; ii < iFontMultiply; ii++)
    {
      yr = y;

      for (j = 0; j < ptrFontHeader->height; j++)
      {
        for (jj = 0; jj < iFontMultiply; jj++)
        {
          my_DrawPixel(xr, yr, _textBackColor);
          yr++;
        }

        if (bbLineSpace)
        {
          for (jj = 0; jj < iFontMultiply; jj++)
          {
            my_DrawPixel(xr, yr, _textBackColor); // j je pix na radku "pod"
            yr++;
          }
        }
      }

      xr++;
    }
  }
#endif
}

bool DISP_Fill(uint32_t color)
{
  int r, c;

  if (my_DrawPixel)
  {
    for (r = 0; r < pixelsHeight; r++)
    {
      for (c = 0; c < pixelsWidth; c++)
        my_DrawPixel(c, r, color);
    }

    return true;
  }
  else
    return false;
}

void DISP_DrawPixel(int x, int y, uint32_t color)
{
  if (my_DrawPixel)
    (*my_DrawPixel)(x, y, color);
}

void DISP_DrawLine(int x0, int y0, int x1, int y1, uint32_t color)
{
  int dx = (x0 < x1) ? (x1 - x0) : (x0 - x1), sx = (x0 < x1) ? 1 : -1;
  int dy = (y0 < y1) ? (y1 - y0) : (y0 - y1), sy = (y0 < y1) ? 1 : -1;
  int err = ((dx > dy) ? dx : -dy) / 2, e2;

  for (; ; )
  {
    if ((x0 == x1) && (y0 == y1))
      break;

    (*my_DrawPixel)(x0, y0, color);

    e2 = err;
    if (e2 > -dx)
    {
      err -= dy; x0 += sx;
    }

    if (e2 < dy)
    {
      err += dx; y0 += sy;
    }
  }
}

void DISP_DrawHLine(int x0, int y0, int w, uint32_t color)
{
  //TODO swap if x0 > x1
  for (; w; w--, x0++)        //!! dirty C-style trick
    (*my_DrawPixel)(x0, y0, color);
}

void DISP_DrawVLine(int x0, int y0, int h, uint32_t color)
{
  for (; h; h--, y0++)        //!! dirty C-style trick
    (*my_DrawPixel)(x0, y0, color);
}


void DISP_DrawRect(int x, int y, int w, int h, uint32_t color)
{
  DISP_DrawLine(x, y, x + w, y, color);
  DISP_DrawLine(x + w, y, x + w, y + h, color);
  DISP_DrawLine(x + w, y + h, x, y + h, color);
  DISP_DrawLine(x, y + h, x, y, color);
}

void DISP_FillRect(int x, int y, int w, int h, uint32_t color)
{
  int ww = w, xx = x;

  for(; h; h--)
  {
    x = xx;

    for(w = ww; w; w--)
    {
      (*my_DrawPixel)(x, y, color);
      x++;
    }

    y++;
  }
}

void DISP_DrawCircle(int centerX, int centerY, int radius, uint32_t color)
{
  int d = (5 - radius * 4) / 4;
  int x = 0;
  int y = radius;

  do
  {
    // ensure index is in range before setting (depends on your image implementation)
    // in this case we check if the pixel location is within the bounds of the image before setting the pixel
    if (((centerX + x) >= 0) && ((centerX + x) <= (pixelsWidth - 1)) && ((centerY + y) >= 0) && ((centerY + y) <= (pixelsHeight - 1))) (*my_DrawPixel)(centerX + x, centerY + y, color);
    if (((centerX + x) >= 0) && ((centerX + x) <= (pixelsWidth - 1)) && ((centerY - y) >= 0) && ((centerY - y) <= (pixelsHeight - 1))) (*my_DrawPixel)(centerX + x, centerY - y, color);
    if (((centerX - x) >= 0) && ((centerX - x) <= (pixelsWidth - 1)) && ((centerY + y) >= 0) && ((centerY + y) <= (pixelsHeight - 1))) (*my_DrawPixel)(centerX - x, centerY + y, color);
    if (((centerX - x) >= 0) && ((centerX - x) <= (pixelsWidth - 1)) && ((centerY - y) >= 0) && ((centerY - y) <= (pixelsHeight - 1))) (*my_DrawPixel)(centerX - x, centerY - y, color);
    if (((centerX + y) >= 0) && ((centerX + y) <= (pixelsWidth - 1)) && ((centerY + x) >= 0) && ((centerY + x) <= (pixelsHeight - 1))) (*my_DrawPixel)(centerX + y, centerY + x, color);
    if (((centerX + y) >= 0) && ((centerX + y) <= (pixelsWidth - 1)) && ((centerY - x) >= 0) && ((centerY - x) <= (pixelsHeight - 1))) (*my_DrawPixel)(centerX + y, centerY - x, color);
    if (((centerX - y) >= 0) && ((centerX - y) <= (pixelsWidth - 1)) && ((centerY + x) >= 0) && ((centerY + x) <= (pixelsHeight - 1))) (*my_DrawPixel)(centerX - y, centerY + x, color);
    if (((centerX - y) >= 0) && ((centerX - y) <= (pixelsWidth - 1)) && ((centerY - x) >= 0) && ((centerY - x) <= (pixelsHeight - 1))) (*my_DrawPixel)(centerX - y, centerY - x, color);
    if (d < 0)
    {
      d += 2 * x + 1;
    }
    else
    {
      d += 2 * (x - y) + 1;
      y--;
    }
    x++;
  } while (x <= y);
}

void DISP_FillCircle(int x0, int y0, int radius, uint32_t color)
{
  int f = 1 - radius;
  int ddF_x = 0;
  int ddF_y = -2 * (int)radius;
  int x = 0;
  int y = (int)radius;
  int t1, t2;

  (*my_DrawPixel)(x0, y0 + radius, color);
  t1 = y0 - radius;
  (*my_DrawPixel)(x0, (t1 > 0) ? t1 : 0, color);
  t1 = x0 - radius;
  DISP_DrawLine(x0 + radius, y0, (t1 > 0) ? t1 : 0, y0, color);

  while(x < y)
  {
    if(f >= 0)
    {
        y--;
        ddF_y += 2;
        f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x + 1;

    t1 = x0 - x;
    t2 = y0 - y;
    DISP_DrawLine((t1 > 0) ? t1 : 0, y0 + y, x0 + x, y0 + y, color);
    DISP_DrawLine((t1 > 0) ? t1 : 0, (t2 > 0) ? t2 : 0, x0 + x, (t2 > 0) ? t2 : 0, color);
    t1 = x0 - y;
    t2 = y0 - x;
    DISP_DrawLine((t1 > 0) ? t1 : 0, y0 + x, x0 + y, y0 + x, color);
    DISP_DrawLine((t1 > 0) ? t1 : 0, (t2 > 0) ? t2 : 0, x0 + y, (t2 > 0) ? t2 : 0, color);
  }
}

static void swapInts(int *a, int *b)
{
  register int c = *a;            //TODO use assembler ?
  *a = *b;
  *b = c;
}

void DISP_FillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint32_t color)
{
  int a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1)
  {
    swapInts(&y0, &y1);
    swapInts(&x0, &x1);
  }
  if (y1 > y2)
  {
    swapInts(&y2, &y1);
    swapInts(&x2, &x1);
  }
  if (y0 > y1)
  {
    swapInts(&y0, &y1);
    swapInts(&x0, &x1);
  }

  if (y0 == y2)
  { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if (x1 < a)
      a = x1;
    else
      if (x1 > b)
        b = x1;
    if (x2 < a)
      a = x2;
    else
      if (x2 > b)
        b = x2;
    DISP_DrawHLine(a, y0, b - a + 1, color);
    return;
  }

  int dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0, dx12 =
      x2 - x1, dy12 = y2 - y1, sa = 0, sb = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if (y1 == y2)
    last = y1;   // Include y1 scanline
  else
    last = y1 - 1; // Skip it

  for (y = y0; y <= last; y++)
  {
    a = x0 + sa / dy01;
    b = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;

    if (a > b)
      swapInts(&a, &b);
    DISP_DrawHLine(a, y, b - a + 1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for (; y <= y2; y++)
  {
    a = x1 + sa / dy12;
    b = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;

    if (a > b)
      swapInts(&a, &b);
    DISP_DrawHLine(a, y, b - a + 1, color);
  }
}

void DISP_DrawSpriteMono8(int x, int y, uint8_t *data, int rows, uint32_t foreColor)
{
//   DISP_FillRect(x, y, 8, rows, foreColor);
  if (data == NULL)
    return;

  for (int r = 0; r < rows; r++)
  {
    uint8_t m = 0x80;
    for (int c = 0; c < 8; c++)
    {
      if (data[r] & m)
        DISP_DrawPixel(x + c, y + r, foreColor);

      m >>= 1;
    }
  }
}

