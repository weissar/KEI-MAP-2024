# Shield MBED - LCD 128x64, joystick, akcelerometr, teplomer (I2C)

* LCD je obsluhován řadičem ST7565R, má rozlišení 128x32 pixelů
  * černobílý = 1bit/pixel
  * připojený přes SPI
* Akcelerometr MMA7660A - stejný jako na MAP shield v1
* Teploměr LM75B na I2C (viz. příslušné PDF)

## Knihovní soubory:
* **ST7565R_driver.h** a **ST7565R_driver.c**
  * nízkoúrovňové funkce přístupu k LCD řadiči
  * framebuffer pro data pixelů, pomocný buffer pro kopírování do LCD
  * interně používá časovač TIM4 pro automatická refresh framebuffer -> LCD, přenos přes DMA
* **display_hilevel.h**, **display_hilevel.c**, **font_type.h**, **font_atari_8x8.h** a **font_atari_8x8.c**, **font_lcd_5x7.h** a **font_lcd_5x7.c**
  * obecné funkce pro rastrovou grafiku
  * interně vyžadují jen low-level obdobu funkce PutPixel (zajištěno) v inicializaci ST7565
* Vhodná inicializace:
```C++
 if (!MBED_LCD_init())
    while (1)
      ;

  DISP_GotoXY(0, 0);
  DISP_WriteString("KEI/MAP");      // neni-li nastaven font, pouzije font_atari_8x8
```
  * Pro samotnou práci je možné využít:
```
  DISP_DrawCircle, DISP_DrawLine, DISP_FillRect ... a řada dalších kreslicích funkcí
  DISP_GotoXY, DISP_TextBackColor, DISP_WriteString ... a řada dalších funkcí pro výpis textu
```
* Připojení dalších signálů postupy známými z cvičení KEI/MAP
```
  #define MBED_RGB_RED      GPIOB,4 // D5 = PB4 ... PWM3/1, AF01
  #define MBED_RGB_GREEN    GPIOC,7 // D9 = PC7 ... PWM3/2, AF02
  #define MBED_RGB_BLUE     GPIOA,9 // D8 = PA9 ... PWM1/2, AF01

  #define MBED_JOY_UP       GPIOA,4 // A2 = PA4
  #define MBED_JOY_DOWN     GPIOB,0 // A3 = PB0
  #define MBED_JOY_LEFT     GPIOC,1 // A4 = PC1
  #define MBED_JOY_RIGHT    GPIOC,0 // A5 = PC0
  #define MBED_JOY_CENTER   GPIOB,5 // D4 = PB5 - ! SWD/debug

  #define MBED_POT_1        GPIOA,0 // A0 = PA0 ... ADC1/0
  #define MBED_POT_2        GPIOA,1 // A1 = PA1 ... ADC1/1

  #define MBED_SPEAKER      GPIOB,10 // D6 = PB10 .. PWM2/3,AF01
```
