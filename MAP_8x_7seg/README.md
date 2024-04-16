# 8x 7 segment LED module

Shield doplňuje modul o 2x uživatelské tlačítko a 1x reset (původní je zakryto PCB shieldu)

## Knihovní soubory:
* **stm_systick.h** a **stm_systick.c**
  * zapouzdřuje práci se SysTickem
  * není potřeba řešit v main.c, stačí zavolat InitSystickDefault();
* **MAX7219_single.h** a **MAX7219_single.c**
  * POZOR - nemíchat s MAX7219_double, ten ovládá dvojici "MAXů"
  * Vhodná inicializace:

```C++
  if (!MAX7219_InitHW())
    while(1)
      ;

        // X0xx = NoOP
        // X1xx - X9xx = Digit 0 .. 7, MSB - DP A B C D E F G
  MAX7219_Send16b(0x0f00);    // XFxx = display test, xx00 = normal, xx01 = test
  MAX7219_Send16b(0x0b07);    // XBxx = scan limit, xxXb = 0 = digit 0 .. 7 = digit 0,1,2..7
  MAX7219_Send16b(0x0900);    // X9xx = decode-mode, xx00 = no decode
  MAX7219_Send16b(0x0c01);    // XCxx = shutdown register, xx01 = normal operation
  MAX7219_Send16b(0x0aff);    // XAxx = intensity, xxXa = a = 1 .. 31 / 32 (step 2)

  MAX7219_Send16b(0x0f00);
```
  * Pro práci je možné využít:
```
  MAX7219_SendDataView ... na pozici 0..7 zapíše 8-bitovou hodnotu
  MAX7219_Send16b ... přímý zápis 16-bitové hodnoty do MAX7219 - viz. datasheet
  to7seg ... pro snažší výpis číslic 0..9 připravená kódovací tabulka na 7-segmentové zobrazení 
```

[Stránka produktu](https://www.laskakit.cz/led-displej-7-segmentovy--8-znaku-max7219-cerveny/) na laskakit.cz

## Několik postřehů pro případnou vlastní tvorbu:
* Je vhodné využít blok SPI komunikace - viz. časování v DS:
  * Rámec začíná CS = 0
  * Přenáší se 16-bitové slovo, pro jednoduchost jako 2 8-bitová po sobě (i když SPI na STM32 umí 16-bitový režim)
  * Případně další 16b slova
  * Na konci CS = 1
* Ověřená inicializace:
  * 0x0f00 = display test - normal operation - viz. Table 10
  * 0x0b07 = scan limit - 8 digits/lines - viz. Table 8
  * 0x0900 = decode mode - no decode - viz. Table 4
  * 0x0c01 = shutdown register - normal op. - viz. Table 3
  * 0x0a08 = intensity - 17/32 - viz. Table 7
* Nedoporučuji používat nějaký decode mode, nebylo by pak možné zobrazit libovolnou bitovou kombinaci - např. pro pseudo-písmena
