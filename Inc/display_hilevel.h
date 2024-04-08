#ifndef _DISPLAY_HILEVEL_H_
#define _DISPLAY_HILEVEL_H_

#include <stdint.h>
#include <stdbool.h>

#ifndef NULL
#define NULL ((void *)0)
#endif

void SetHiLevelDisp(int pixWidth, int pixHeight, void *ptrDrawPixel);

int DISP_GetWidth(void);
int DISP_GetHeight(void);


void DISP_DrawPixel(int x, int y, uint32_t color);
void DISP_DrawLine(int x0, int y0, int x1, int y1, uint32_t color);
void DISP_DrawHLine(int x0, int y0, int w, uint32_t color);
void DISP_DrawVLine(int x0, int y0, int h, uint32_t color);
void DISP_DrawRect(int x, int y, int w, int h, uint32_t color);
void DISP_FillRect(int x, int y, int w, int h, uint32_t color);
void DISP_DrawCircle(int centerX, int centerY, int radius, uint32_t color);
void DISP_FillCircle(int x0, int y0, int radius, uint32_t color);
void DISP_FillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint32_t color);

void DISP_TextForeColor(uint32_t u);
void DISP_TextBackColor(uint32_t u);
void DISP_WriteCharXY(int x, int y, char c);
void DISP_WriteChar(char c);
void DISP_WriteString(char *cp);
void DISP_GotoXY(int colPix, int rowPix);
// bool DISP_Clear(void);
bool DISP_Fill(uint32_t color);

void DISP_DrawSprite(int x, int y, void *pbuf);

typedef enum {font_atari_8x8, font_lcd_5x7,
  // font_thin_8x8,
  // font_system_5x7,
  // font_wendy_3x5,
  //font_newbasic_3x6
} eFontsAvailable;
bool DISP_SetFont(eFontsAvailable fnt);
bool DISP_SetFontMultiply(int m);
int DISP_GetCharPerRow(void);
int DISP_GetRowPerDisp(void);
int DISP_GetFontWidth(void);
int DISP_GetFontHeight(void);

void DISP_DrawSpriteMono8(int x, int y, uint8_t *data, int rows, uint32_t foreColor);

#endif // _DISPLAY_HILEVEL_H_
