# 8x 7 segment LED module

[Stránka produktu](https://www.laskakit.cz/led-displej-7-segmentovy--8-znaku-max7219-cerveny/) na laskakit.cz

Několik postřehů:
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
