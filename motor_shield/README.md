# Motor shield

## Připravená knihovna (předběžně stále ve vývoji)

### Práce s DC motory

* **stm_core_addon.h** a **stm_core_addon.c**
  * rozšíření z "běžného" stm_core o funkce pro zjištění busclock a timerclock
  * interně používané při inicializaci časovačů
* Hlavní soubory k shieldu - **Shield_L293D.h** a **Shield_L293D.c**
* **Momentálně podporuje pouze "stepper 2" a "servo 1" (teoreticky možno provozovat najednou)**
* **Nová verze - podporuje motory 1-4, neměla by dovolit kolizi mezi např. motory a "krokáčem"**
  * Některé PWM výstupy jsou nevhodně zkombinované pro Nucleo, zatím dále neřešeno
* Doporučená inicializace pro motor (1..4)
```C++
...
  if (!motors_motor_enable(1, 1500))    // 1.5kHz PWM
  {
    puts("Fail init motor 1");
    while (1)
      ;
  }
```
* Nastavení rychlosti v rozsahu 0-100 (dle inicializace PWM) a směru otáčení pro zvolený motor:
```C++
    if (!motors_motor_run(2, speed, dir1))
      puts("Set motor speed");
    else
      printf("Motor speed: %d\n", speed);
```
* Připojení senzoru otáčení na libovolný GPIO pin na vnější řadě "Nucleo konektorů" a nastavit jako "vstup"
* Několik postřehů ke schématu zapojení (viz. také začátek .C souboru):
  * Řídící signály pro 2x dvojitý H-můstek L293D jsou připojené na '595
    * Sériově paralelní převodník, piny CLK = B5, SER = A9, EN = A8, LATCH = A6
    * Stejný je použít na shieldu v2 pro SPI LED
    * Výstupy pro "můstky":
      * QA - M3A
      * QB - M2A
      * QC - M1A
      * QD - M1B
      * QE - M2B
      * QF - M4A
      * QG - M3B
      * QH - M4B
  * Další signály (typicky PWM):
    * D2 - PA10 - JP3 - ??
    * D3 - PB3 - PWM2B - motor 2 = PWM2/2 (AF01)
    * D5 - PB4 - PWM0B - motor 3 = PWM3/1 (AF02)
    * D6 - PB10 - PWM0A - motor 4 = PWM2/3 (AF01)
    * D9 - PC7 - PWM1A - servo 2 = PWM3/2 (AF02)
    * D10 - PB6 - PWM1B - servo 1 = PWM4/1 (AF02)
    * D11 - PA7 - PWM2A - motor 1 = PWM1/1N (AF01), TIM3CH2 (AF02)
* Nejjednodušší připojení motoru 1:
![Motor 1](./motorr_1_to_shield.jpg)

### Práce s krokovým motorem
* Nastavení pinů/signálů - !! POZOR, jen servo 2, tj. M3A+B a M4A+B
```C++
...
  if (!motors_stepper_enable(2))    //!! only stepper 2
  {
    puts("Fail init stepper 2");
    while (1)
      ;
  }
```
* Nastevení kombinace výstupů - 4 bity odpovídají 4 "koncům cívek"
```C++
...
  if (!motors_stepper_set_bits(2, 0x08))    // stepper 2, bits to output 1000
  {
    puts("Fail set outputs to stepper");
  }
```

* Signály/vývody krokového motoru - viz. [zdroj](https://components101.com/motors/28byj-48-stepper-motor) - upraveno pro "stepper 2"
![Stepper Wiring](./28BYJ-48-Pinout-Wirings.png)
* Příklad zapojení - střed cívek zapojen na GND:
![Stepper real example](./stepmotor_to_shield.jpg)

## Další zdroje informací pro možný vlastní vývoj
Produktové stránky na laskakit.cz:
* [Motor shield](https://www.laskakit.cz/arduino-4-kanalovy-motor-driver-shield-l293d/)
* ["žlutý" motor](https://www.laskakit.cz/tt-motor-s-prevodovkou-plastove-prevody/)
* [Krokový motor 28BYJ-48](https://www.laskakit.cz/krokovy-motor-28byj-48/)
* [Mikroservo kontinuální](https://www.laskakit.cz/plastove-micro-servo-sg90-9g--kontinualni/)
* [Mikroservo](https://www.laskakit.cz/plastove-micro-servo-sg90-9g--180/)
* [Snímač otáček](https://www.laskakit.cz/fotoelektricky-snimac-otacek/)
