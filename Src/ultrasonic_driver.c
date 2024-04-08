#include "ultrasonic_driver.h"

static bool _initOk = false;

#define _US_TIMER_OBJ   TIM4        //TODO make configurable !!

static bool _US_InitHW(void)
{
  STM_SetPinGPIO(SHIELD_SONAR_TRIG, ioPortOutputPP);    //TODO configurable ?
  STM_SetPinGPIO(SHIELD_SONAR_ECHO, ioPortInputPU);

  switch((uint32_t)_US_TIMER_OBJ)
  {
    case  (uint32_t)TIM4:
      if (!(RCC->APB1ENR & RCC_APB1ENR_TIM4EN))
      {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM4RST;
      }
      break;
    default:
      return false;
  }

  _US_TIMER_OBJ->CR1 = 0
      // ~TIM_CR1_DIR;
      // | TIM_CR1_ARPE
      ;
  _US_TIMER_OBJ->PSC = STM_GetTimerClock(4) / 1E6 - 1;   // set prescaler to 1us steps
  _US_TIMER_OBJ->ARR = 50000;         // 50ms
  _US_TIMER_OBJ->CNT = TIM4->ARR - 10;

  _US_TIMER_OBJ->CR1 |= TIM_CR1_CEN;  // ARPE / neARPE, musi se aspon jednou nastavit TIF :-/

  for(int i = 0; i < 1000; i++)       // chvulka cekani aby to stihnlo pretect
    __asm("NOP");

  return true;
}

uint32_t US_MeassureBlocking(void)
{
  if (!_initOk)
  {
    _US_InitHW();
    _initOk = true;
  }

  _US_TIMER_OBJ->CR1 &= ~TIM_CR1_CEN;
  _US_TIMER_OBJ->CNT = 0;

  GPIOWrite(SHIELD_SONAR_TRIG, 1);

  _US_TIMER_OBJ->CR1 |= TIM_CR1_CEN;     // start

  while(_US_TIMER_OBJ->CNT < 40)         // 40us pulse
    ;

  GPIOWrite(SHIELD_SONAR_TRIG, 0);

  while(!GPIORead(SHIELD_SONAR_ECHO) && (_US_TIMER_OBJ->CNT < 40000))   // begin pulse or 40ms
    ;

  uint32_t dist = _US_TIMER_OBJ->CNT;

  while(GPIORead(SHIELD_SONAR_ECHO) && (_US_TIMER_OBJ->CNT < 40000))   // end of pulse or 40ms
    ;

  _US_TIMER_OBJ->CR1 &= ~TIM_CR1_CEN;    // stop
  dist = _US_TIMER_OBJ->CNT - dist;

  return dist;
}

float US_ConvertToMM(uint32_t width_ms)
{
  return width_ms / 5.8;
}
