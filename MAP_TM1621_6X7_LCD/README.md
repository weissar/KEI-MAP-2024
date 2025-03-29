# LCD Display 6x 7 -segment - HT1621 Controller

## Knihovní soubory:
* **HT1621_LCD.h** a **HT1621_LCD.c**
* Vhodná inicializace:
```C++
  if (!HT1621_Init(6))
    while (1)
      ;

  WaitMs(1000);     // cekej na dokonceni inicializace
```
* Pro práci je možné využít:
```
  HT1621_Init ... inicializace, logicky je pocet číslic = 6
  HT1621_Backlight ... zapnutí/vypnutí podsvícení
  HT1621_WriteDigitData ... zapíše na pozici 8-bitovou hodnotu, kde bity odpovídají segmentům LCD 
  HT1621_WriteDigitNum ... zobrazí na pozici číslici 0..9 (interně má tabulku pro rozsvícení segmentů)
  HT1621_SetHiBit ... nejvyšší bit není součástí číslice, ale symbolem tečky nebo baterky - zde je možno bitově nastavit, zda se ve funkcích WriteDigitNum a WriteDigitChar rozsvítí
  HT1621_WriteDigitChar ... zapíše na pozici bitový vzor odpovídající ASCII znaku (vzhled definován tabulku, zatím jen ' ' a '-' )
```
* Na shieldu jsou dále tlačítka na GPIOB,4 a GPIOB,5 (jako na MAP-shieldu)
* Podsvícení je připojené na GPIOA,6 - ovládané pouze 1 a 0, ale je možné využít i PWM - pak je potřeba po inicializaci nastavit GPIO na PWM a nepoužívat funkci HT1621_Backlight().

## Několik postřehů pro případnou vlastní realizaci:

[Stránka produktu](https://www.laskakit.cz/6-mistny-sedmisegmentovy-lcd-displej-2-4--ht1621--bily/) na webu laskakit.cz

Řadič HT1621 komunikuje ve stylu SPI, ale vyžaduje speciální posílání bitové sekvence, proto je využit podobný kód jako "ruční generování SPI" - signály se nastavovují pomocí GPIOWrite apod.
* Časování signálů - viz. DS - [formát PDF](./HT1621.pdf)
* Zapojení signálů viz. [schéma PDF](./MAP_6X7_SEGMENT_DISP.PDF)

