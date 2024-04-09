# LED modul 16x8 řízený dvojicí MAX7219 - červené LED

## Knihovní soubory:
* MAX7219_double.h a MAX7219_double.c (pozor, nekombinovat s MAX..._single)
  * Obsaují inicializaci a základní funkce pro použití
  * Vhodná inicializace:
```C++
  if (!MAX7219_InitHW())          // inicializace SPI a pinu pro komunikaci
    while (1)
      ;

  if (!MAX7219_Dbl_InitSW())      // obsahuje inicializaci MAXu
    while (1)
      ;
```
  * Pro práci je možné využít:
```
  MAX7219_Dbl_PutPixel ... nastaví 1 pixel ve framebufferu na svítí(true)/nesvítí(false)
  MAX7219_Dbl_DrawNum_3x5 ... výpis 1 znaku na pozici, font 3x5 obsahuje pouze číslice 0..9
  MAX7219_Dbl_Refresh ... překreslení obsahu framebufferu z paměti do řadičů MAX7219 (= zobrazení)
  !! font_data_3x5 ... pole v knihovně, definice znaků, možno přidat písmena apod.
```

## Několik postřehů pro případnou vlastní realizaci:

Na desce jsou použity 2x řadiče MAX7219 a modul 8x8 LED ze [stavenice](https://www.laskakit.cz/stavebnice-8x8-cervena-led-matice-smd-max7219/), které jsou zapojeny v daisy-chain zapojení - viz. schéma.

* Je vhodné využít blok SPI komunikace - viz. časování v DS:
  * Rámec začíná CS = 0
  * Přenáší se 16-bitové slovo, pro jednoduchost jako 2 8-bitová po sobě (i když SPI na STM32 umí 16-bitový režim)
  * Případně další 16b slova
  * Na konci CS = 1
* Příkazy pro "druhý" řadič přeposílá ten první až po přijetí "svých" 16 bitů, proto je vhodné jako první poslat povel NOP - viz. DS - "Table 2"
* Ověřená inicializace:
  * 0x0f00 = display test - normal operation - viz. Table 10
  * 0x0b07 = scan limit - 8 digits/lines - viz. Table 8
  * 0x0900 = decode mode - no decode - viz. Table 4
  * 0x0c01 = shutdown register - normal op. - viz. Table 3
  * 0x0a08 = intensity - 17/32 - viz. Table 7

