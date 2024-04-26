# Modul TFT LCD 128x128 a ultrazvukový senzor vzdálenosti

* LCD je obsluhován řadičem ILI9153, má rozlišení 128x128 pixelů
  * 16-bitové barvy pixelů v kódování 5-6-5
  * připojený přes SPI, LED podsvícení může být alternativně řízení i PWM (neni v knihovně)
* HC-SR04 modul UZ měření vzdálenosti 

## Knihovní soubory:
* **stm_systick.h** a **stm_systick.c**
  * nahrazují použití SysTick v main.c
  * funkčnost využitá i v modulu obsluhy LCD
* **stm_core_addon.h** a **stm_core_addon.c**
  * rozšíření z "běžného" stm_core o funkce pro zjištění busclock a timerclock
  * interně používané při inicializaci LCD
* **ILI9163C_driver.h** a **ILI9163C_driver.c**
  * nízkoúrovňové funkce přístupu k LCD řadiči
  * framebuffer pro data pixelů, pomocný buffer pro kopírování do LCD
  * použití SPI s 16-bitovým přenosem a DMA pro blokový přenos
* **display_hilevel.h**, **display_hilevel.c**, **font_type.h**, **font_atari_8x8.h** a **font_atari_8x8.c**, **font_lcd_5x7.h** a **font_lcd_5x7.c**
  * obecné funkce pro rastrovou grafiku
  * interně vyžadují jen low-level obdobu funkce PutPixel (zajištěno)
* Vhodná inicializace:
```C++
 if (!ILI9163C_Init())
    while (1)
      ;

  ILI9163C_Rotate180(true);       // moznost otocit obsah LCD o 180st, meni chovani interniho PutPixel
```
  * !! Pozor: není automatický přepis obsahu FB na LCD, nutno volat pravidelně (v podstatě co nejčastěji, např. po 1 nebo 2 msec) ručně:
    * Je potřeba ověřit, že neprobíhá žádné předchozí kopírování bufferů - funkce **ILI9163C_InProgress** vrací false, pokud se nic neděje a je tedy možno vykreslit nový obsah "scény" (plochy displeje) do paměti
    * Následně je potřeba ověřit, že neprobíhá přenos - funkce **ILI9163C_FrameFinish** vrací příznak "hotovo" a pokud máme vykreslenou scénu do paměti (předchozí krok), je možné spustit kopírování do LCD pomocí funkce **ILI9163C_Refresh**
```C++
      if (!ILI9163C_InProgress())                 // je volny FB pro kresleni ?
      {
        DrawScene();
        _sceneReady = true;
      }

      if (ILI9163C_FrameFinish() && _sceneReady)   // je hotovo v knihovne a pripravena data ?
      {
        ILI9163C_Refresh();
        _sceneReady = false;
      }
```
  * Pro samotnou práci je možné využít:
```
  ILI9163C_ClearFB ... vyplní framebuffer zvolenou barvou
  DISP_DrawCircle, DISP_DrawLine, DISP_FillRect ... a řada dalších kreslicích funkcí
  DISP_GotoXY, DISP_TextBackColor, DISP_WriteString ... a řada dalších funkcí pro výpis textu
```
* **ultrasonic_driver.h** a **ultrasonic_driver.c**
  * jednoduché funkce pro měření času (vzdálenosti)
  * interně využívá časovač TIM4
  * Inicializace není potřeba, zajistí si modul automaticky při prvním použití
  * Pro práci je možné využít:
```
  US_MeassureBlocking ... vrací čas odezvy - !! je blokující, pro vzdálený předmět může trvat až 40ms !!
  US_ConvertToMM ... pomocná funkce pro přepočet na [mm], vychází z rychlosti zvuku
```
