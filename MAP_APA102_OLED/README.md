# Shield OLED 128x64 px + 10x APA102

Shield obsahuje:
* OLED displej 128x64 pixelů, připojený přes I2C sběrnici
* 10x RGB LED APA102 (SPI komunikace - synchronní s hodinami)
* 2x tlačítko

## Knihovní soubory:
* stm_systick.h a stm_systick.c
  * zapouzdřuje práci se SysTickem
  * není potřeba řešit v main.c, stačí zavolat InitSystickDefault();
* APA102_RGB.h a APA102_RGB.c
  * Funkce pro přístup k SPI-RGB + generátor efektu duhy
  * Vhodná inicializace:
```C++
  if (!APA102_init(10))
    while(1)
      ;

  APA102_SetIntesity(1);          // jinak to opravdu hodne sviti

  for(int i = 0; i < 10; i++)     // pro vsechny RGB LED
    APA102_LED(i, 0, 0, 0);       // black = nothing

  APA102_Refresh();               // update
```
  * Pro práci je možné využít:
```
  APA102_RainbowNext ... zobrazí další prvky ze sekvence duhy (interne predpocitano 32 polozek)
  APA102_LED ... nastavi pro vybranou LED hodnoty RGB složek
  APA102_Refresh ... změny je potřeba vždy propagovat do HW (zde interně přes SPI)
```
* OLED_SH1106.h a OLED_SH1106.c
  * Funkce pro low-level práci s OLED řadičem SH1106 (podobný rozšířenější SSD1305)
* display_hilevel.h a display_hilevel.c, font_type.h, font_atari_8x8.h a font_atari_8x8.c, font_lcd_5x7.h a font_lcd_5x7.c
  * Vhodná inicializace:
```C++
  if (!OLED_SH1106_Init())      // inicializace
    while(1)                    // v pripade problemu zustane zde
      ;

  DISP_Fill(0);                 // vycisteni FB

  DISP_SetFont(font_atari_8x8); // vzchoyi font
  DISP_SetFontMultiply(1);      // a faktor zvetseni fontu
```
  * Pro práci je možné využít:
```
  OLED_SH1106_UpdateContent ... po změně obsahu framebufferu nutno překopírovat do displeje
  DISP_GotoXY ... umístí kurzor na pozici
  DISP_WriteString ... výpis řetězce aktuálním fontem
  DISP_DrawRect ... vykreslení obdélníku
  ... a řada dalších hi-level funkcí pro displeje
```
