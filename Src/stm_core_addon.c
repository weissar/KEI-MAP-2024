#include "stm_core_addon.h"

/**
 * \fn uint32_t STM_GetTimerClock(int)
 * \brief Calculate real clock based on APBx for selected timer
 * \remark Predefined for some STM32xx models, other must be added
 *
 * \param timerNum Number of TIMx
 * \return Timer input clock
 */
uint32_t STM_GetTimerClock(int timerNum)
{
  uint32_t apbdiv = 0, timerClock = SystemCoreClock;

#if defined(STM32F411xE) // || defined(STM32F413xx)  // || defined ...
  switch(timerNum)
  {
    case 1:
    case 9:
    case 10:
    case 11:
      apbdiv = RCC->CFGR & RCC_CFGR_PPRE2;    // 0x0000E000 - keep bits 15..13
      apbdiv >>= 13;                          // move right
      break;
    case 2:
    case 3:
    case 4:
    case 5:
      apbdiv = RCC->CFGR & RCC_CFGR_PPRE1;    // 0x00001C00 - keep bits 12:10
      apbdiv >>= 10;
      break;
    default:
      //TODO emit error for unknown Timer
      break;
  }
#else
#error Valid controller not set - GetTimerClock
#endif

  if ((apbdiv & 0x04) == 0)                   // MSB of that 3 bits is 0 ?
    timerClock = SystemCoreClock;             // not divided, eg. x1
  else
    timerClock = 2 * (SystemCoreClock >> ((apbdiv & 0x03) + 1));   // lower 2 bits is divider

  return timerClock;
}

/**
 * \fn uint32_t STM_GetBusClock(eBusClocks)
 * \brief Calculate bus clock (APBx, AHBx) from main clock settings
 * \remark Limited to predefined controller types, can be extended
 *
 * \param clk One of eBusClocks enum values
 * \return Clock in Hz
 */
uint32_t STM_GetBusClock(eBusClocks clk)
{
  uint32_t bitval = 0;
  uint32_t divider = 1;

#if defined(STM32F411xE) || defined(STM32F413xx)  // || defined ...
  switch(clk)
  {
    case busClockAHB:
      bitval = (RCC->CFGR & (0x0f << 4)) >> 4;   // HPRE [7:4] to lower 4 bits
      if (bitval & 0x8)           // 1xxx
        divider = 1 << ((bitval & 0x07) + 1);   // 0 = /2, 1 = /4
      else
        divider = 1;              // 0xxx = not divided
      break;
    case busClockAPB1:
    case timersClockAPB1:         // x2
      bitval = (RCC->CFGR & (0x07 << 10)) >> 10; // PPRE1 [12:10] to lower 3 bits
      if (bitval & 0x4)           // 1xx
        divider = 1 << ((bitval & 0x03) + 1);   // 0 = /2, 1 = /4
      else
        divider = 1;              // 0xx = not divided

      break;
    case busClockAPB2:
    case timersClockAPB2:         // the same
      bitval = (RCC->CFGR >> 13) & 0x07; // PPRE2 [15:13] to lower 3 bits
      if (bitval & 0x4)           // 1xx
        divider = 1 << ((bitval & 0x03) + 1);   // 0 = /2, 1 = /4
      else
        divider = 1;              // 0xx = not divided
      break;
    default:
      return 0;
  }

  SystemCoreClockUpdate();      // for sure recalculate SystemCoreClock

  if (((clk == timersClockAPB1) || (clk == timersClockAPB1)) && (divider > 1))
    return SystemCoreClock / divider * 2;
  else
    return SystemCoreClock / divider;
#else
#error Valid controller not set - GetBusClock
#endif
}
