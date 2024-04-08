#include "stm_systick.h"

volatile uint32_t _ticks = 0;
volatile uint32_t _interval = 1;

void SysTick_Handler(void)
{
  _ticks++;
}

void WaitMs(uint32_t ms)
{
  ms += _ticks;
  while(_ticks < ms)
    ;
}

void InitSystickDefault(void)
{
  InitSystick(1);
}

void InitSystick(uint32_t intervalMs)
{
  SystemCoreClockUpdate();               // for sure

  _interval = intervalMs;                // internally store for waiting calculation
  SysTick_Config(SystemCoreClock / 1000 / _interval);  // set to generate interrupt every X ms
}

