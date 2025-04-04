# LCD_keypad shield

[Stránka produktu](https://www.laskakit.cz/arduino-1602-lcd-klavesnice-shield/) na laskakit.cz

## Knihovní soubory:
* **HD44780_LCD.h** a **HD44780_LCD.c**
  * Obsaují inicializaci a základní funkce pro použití
  * Vhodná inicializace:
```C++
  if (!HD44780_init_hw() || !HD44780_init_sw())
    while (1)
      ;

  HD44780_set_cursor(0,  1);        // zacatek 2. radku
  HD44780_putstring("KEI/MAP");
```
  * Pro práci je možné využít:
```
  HD44780_set_cursor ... zobrazí další prvky ze sekvence duhy (interne predpocitano 32 polozek)
  HD44780_putchar ... výpis 1 znaku na pozici kurzoru (+ automatický posun kurzoru)
  HD44780_putstring ... výpis řetězce, zřejmě předtím potřeba použít sprintf apod.
```
* Na pinu PA0 připojeny přes R tlačítka, zjištění stisknutého pomocí A/D převodníku
  * Pozor na případný vícenásobná stisk
  * Při zpracování připravit "hranice mezi hodnotami" a při měření hledat nejbližší

## Základní vlastnosti (užitečné informace pro vlastní realizaci SW přístupu):
* Řadič displeje je kompatibilní s oblíbeným HD44780
* Připojení 4-bitové (8-bitová data nutno posílat na poloviny - viz. DS)
* Tlačítka jsou připojena na vstup A/D a fungují jako "dělič" = pro každé tlačítko je čtena "analogová" hodnota ... pozor, chce to testovat interval, měřená hodnota není úplně přesná/opakovatelná

Třebaže je řadič HD44780 velmi známý a dokumentovaný, ne všechny postupy fungují. Praktické postřehy:
* Doba trvání pulsu signálu E by měla být dle DS min. 450ns, doporučuji několik us (např. několikrát for cyklus)
* Mezi zápisem "horních" 4 bitů a "dolních" je lepší také dát krátké čekání
* Puls na E je potřeba udělat po nastavení datových výstupů (sestupná hrana E nesmí být ihned po nastavení dat)
* Je vhodné si udělat funkci pro zápis 4-bitu (signálem E) a funkci pro zápis celé 8-bitové hodnoty (jak pro příkazy, tak pro data)

Ověřená sekvence pro inicializaci:
* Přepnutí do režimu příkazů = RS = 0
* Zapsat 4-bitově hodnotu 0011
* Počkat cca 10ms
* Zapsat znovu 4-bitově hodnotu 0011
* Počkat cca 10ms
* Zapsat potřetí 4-bitově hodnotu 0011
* Počkat cca 10ms
* Zapsat 4-bitově hodnotu 0010 = přepnutí do 4-bitové komunikace
* Počkat 5ms
* Dále už se používá "normální" 8-bitová komunikace, např. příkazy dle "Table 6" v DS:
  * 0x10 = žádný posun obsahu displeje
  * 0x0F = display ON, cursor ON, cursor blinking
  * 0x01 = display clear
  * 0x80 = kurzor na pozici 0 na řádku 0

**Všechny příkazy trvají minimálně 37us podle DS kromě 0x01, který má dáno min. 1.52ms. Je ale lepší čekat po každém třeba 5ms, pro jistotu. Minimálně během inicializace.**
