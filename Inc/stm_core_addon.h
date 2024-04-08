#ifndef _STM_CORE_ADDON_H
#define  _STM_CORE_ADDON_H

#include "stm_core.h"

typedef enum { busClockAHB,
  busClockAPB1, busClockAPB2,
  timersClockAPB1, timersClockAPB2 } eBusClocks;

uint32_t STM_GetTimerClock(int timerNum);
uint32_t STM_GetBusClock(eBusClocks clk);

#endif // _STM_CORE_ADDON_H
