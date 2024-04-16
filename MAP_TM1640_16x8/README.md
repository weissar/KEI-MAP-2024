# LED Matrix 16x8 - TM1640 Controller

## Knihovní soubory:
* **TM1640_LED.h** a **TM1640_LED.c**
* Vhodná inicializace:
```C++
  if (!TM1640_Init())
    while (1)
      ;

  TM1640_Refresh();           // write FB to display
  TM1640_DispOn();            // enable view
```
* Pro práci je možné využít:
```
  TM1640_SetPixel ... nastaví 1 pixel ve framebufferu na svítí(true)/nesvítí(false)
  TM1640_Refresh ... překreslení obsahu framebufferu z paměti do řadiče TM1640 (= zobrazení)
```
* Na shieldu je dále akcelerometr LIS2DE12
  * Adresa pro I2C je 0x32, pro spuštění převodu stačí zapsat do CTRL_REG1, viz.:
```
  #define ADR_LIS2DE12    0x32              // 0011 00xB  ... x = SAD pin = log 1, B = R/w

  I2C1_WriteByte(ADR_LIS2DE12, 0x20, 0x5f);     // CTRL_REG1 - see 8.6
```
  * Hodnoty "zrychlení" jsou v registrech 0x29, 0x2b a 0x2d
  * **Nutno vyčítat jednotlivě, nelze jako blok**
  * Jsou to 8-bitová čísla se znaménkem v základním rozsahu +-2g

## Několik postřehů pro případnou vlastní realizaci:

[Stránka produktu](https://www.laskakit.cz/lilygo-ttgo-led-matrix-displej/) na laskakit.cz

Reálně je to známé jako [LilyGO TTGO 16x8 LED Matrix displej](http://www.lilygo.cn/prod_view.aspx?TypeId=50033&Id=1176)

Řadič TM1640 komunikuje ve stylu pseudo-I2C sběrnice = signály DATA a SCK. Není to ale přímo I2C, proto je potřeba využít podobného kódu jako pro "ruční generování SPI" - signály nutno nastavovat pomocí GPIOWrite.

* Časování signálů - viz. DS kap. VI
* Význam příkazů - viz. DS kap. VII

Max. frekvence hodin je 1MHz (kap. V) - pro takt mikrokontroléru 16MHz se asi nemusí řešit.

![Pohled na shield s vyznačením 0,0](./LED-16x8%20SHIELD.jpg)

Další komponenty:
* akcelerometr [LIS2DE12TR](https://www.st.com/en/mems-and-sensors/lis2de12.html)
* Spínačový [joystick](https://tech.alpsalpine.com/e/products/detail/SKRHABE010/) - zobrazeno zapojení vývodů
