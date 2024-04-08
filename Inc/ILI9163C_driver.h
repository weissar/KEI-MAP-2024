#ifndef ILI9163C_DRIVER_H_
#define ILI9163C_DRIVER_H_

#include "stm_core.h"           // own system functions
#include "stm_core_addon.h"     // reuqires special addons - timer clock
#include "display_hilevel.h"

// predefined size of LCD, not checked to real panel resolution
#define LCD_WIDTH   128         //LCD width
#define LCD_HEIGHT  128         //LCD height

bool ILI9163C_Init(void);        //TODO configure rotation etc.

bool ILI9163C_SetWindow(int x1, int y1, int x2, int y2);
bool ILI9163C_ClearFB(uint16_t color);
bool ILI9163C_Refresh(void);
void ILI9163C_Rotate180(bool toRotate);

bool ILI9163C_InProgress(void);
bool ILI9163C_FrameFinish(void);

//TODO respect TFT bit organization and core-endianity
// gggr rrrr bbbb bggg - for 16b access and 16b DMA-to-SPI transfers
#define RGB888(r,g,b)  (((((r) >> 3) & 0x001f) << 8) \
    | ((((g) >> 0) & 0x07) << 0) | (((g >> 5) & 0x07) << 13) \
    | ((((b) >> 3) & 0x1f) << 3))

// base colors, internally used higher 5-6-5 bits
// order: R, G, B
#define TFT16BIT_BLACK       RGB888(   0,   0,   0 )
#define TFT16BIT_NAVY        RGB888(   0,   0, 128 )
#define TFT16BIT_DARKGREEN   RGB888(   0, 128,   0 )
#define TFT16BIT_DARKCYAN    RGB888(   0, 128, 128 )
#define TFT16BIT_MAROON      RGB888( 128,   0,   0 )
#define TFT16BIT_PURPLE      RGB888( 128,   0, 128 )
#define TFT16BIT_OLIVE       RGB888( 128, 128,   0 )
#define TFT16BIT_LIGHTGREY   RGB888( 192, 192, 192 )
#define TFT16BIT_DARKGREY    RGB888( 128, 128, 128 )
#define TFT16BIT_BLUE        RGB888(   0,   0, 255 )
#define TFT16BIT_GREEN       RGB888(   0, 255,   0 )
#define TFT16BIT_CYAN        RGB888(   0, 255, 255 )
#define TFT16BIT_RED         RGB888( 255,   0,   0 )
#define TFT16BIT_MAGENTA     RGB888( 255,   0, 255 )
#define TFT16BIT_YELLOW      RGB888( 255, 255,   0 )
#define TFT16BIT_WHITE       RGB888( 255, 255, 255 )
#define TFT16BIT_ORANGE      RGB888( 255, 165,   0 )
#define TFT16BIT_GREENYELLOW RGB888( 173, 255,  47 )

#endif /* ILI9163C_DRIVER_H_ */
