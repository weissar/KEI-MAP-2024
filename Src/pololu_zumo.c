#include "pololu_zumo.h"

#define POLOLU_ZUMO_YELLOW_LED   GPIOA,5
#define POLOLU_ZUMO_BUTTON_SW3   GPIOA,6

// jumper select A4/D2
#define POLOLU_ZUMO_SENSOR_LED   GPIOC,1
// #define POLOLU_ZUMO_SENSOR_LED   GPIOA,10

// jumper select D3/D6
#define POLOLU_ZUMO_BUZZER    GPIOB,3    // TIM2ch2
//#define ZUMO_BUZZER    GPIOB,10    // TIM2ch3

#define POLOLU_ZUMO_BAT_2_3    GPIOA,1     // ADC1ch1

#define POLOLU_ZUMO_M2PWM    GPIOB,6     // TIM4ch1, LEFT
#define POLOLU_ZUMO_M1PWM    GPIOC,7     // TIM3ch2, RIGHT
#define POLOLU_ZUMO_M2DIR    GPIOA,9
#define POLOLU_ZUMO_M1DIR    GPIOA,8

bool Pololu_Zumo_InitMotors(void)
{
  STM_SetPinGPIO(POLOLU_ZUMO_YELLOW_LED, ioPortOutputPP);
  STM_SetPinGPIO(POLOLU_ZUMO_BUTTON_SW3, ioPortInputPU);

  STM_SetPinGPIO(POLOLU_ZUMO_SENSOR_LED, ioPortOutputPP);

  STM_SetPinGPIO(POLOLU_ZUMO_BUZZER, ioPortOutputPP);

  STM_SetPinGPIO(POLOLU_ZUMO_M1DIR, ioPortOutputPP);
  STM_SetPinGPIO(POLOLU_ZUMO_M1PWM, ioPortAlternatePP);
  STM_SetAFGPIO(POLOLU_ZUMO_M1PWM, 2);     // AF02 = PB6 TIM4_CH1

  STM_SetPinGPIO(POLOLU_ZUMO_M2DIR, ioPortOutputPP);
  STM_SetPinGPIO(POLOLU_ZUMO_M2PWM, ioPortAlternatePP);
  STM_SetAFGPIO(POLOLU_ZUMO_M2PWM, 2);     // AF02 = PC7 TIM3_CH2

  // use TIM4 for LEFT motor
  if (!(RCC->APB1ENR & RCC_APB1ENR_TIM4EN))
  {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM4RST;
  }

  TIM4->CR1 = 0
      | TIM_CR1_DIR       // down
      ;
  TIM4->CR2 = 0;

  TIM4->PSC = STM_GetTimerClock(4) / 5E6 - 1;       // 5us
  TIM4->ARR = 200 - 1;        // 1ms = 1kHz

  TIM4->CCMR1 &= 0xff00;    // lower 8 bits = 0
  TIM4->CCMR1 |= 0          // CC1S = 00 = output
    | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1    // OC1M = 110 = PWM mode 1
//    | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 // OC1M = 111 = PWM mode 2
    | 0;
  TIM4->CCER |= TIM_CCER_CC1E;

  TIM4->CR1 |= TIM_CR1_CEN;

  // use TIM3 for RIGHT motor
  if (!(RCC->APB1ENR & RCC_APB1ENR_TIM3EN))
  {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
  }

  TIM3->CR1 = 0
      | TIM_CR1_DIR       // down
      ;
  TIM3->CR2 = 0;

  TIM3->PSC = STM_GetTimerClock(3) / 5E6 - 1;       // 5us
  TIM3->ARR = 200 - 1;        // 1ms = 1kHz

  TIM3->CCMR1 &= 0x00ff;    // higher 8 bits = 0
  TIM3->CCMR1 |= 0          // CC1S = 00 = output
    | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1    // OC2M = 110 = PWM mode 1
    | 0;
  TIM3->CCER |= TIM_CCER_CC2E;

  TIM3->CR1 |= TIM_CR1_CEN;

  return true;
}

void Pololu_Zumo_Motors(int leftVal, int rightVal)
{
  if (leftVal > 200)     //TODO check against TIM4->ARR
    leftVal = 200;
  if (leftVal < -200)
    leftVal = -200;

  if (rightVal > 200)
    rightVal = 200;
  if (rightVal < -200)
    rightVal = -200;

  GPIOWrite(POLOLU_ZUMO_M2DIR, leftVal < 0);
  TIM4->CCR1 = (leftVal > 0) ? leftVal : -leftVal;

  GPIOWrite(POLOLU_ZUMO_M1DIR, rightVal < 0);
  TIM3->CCR2 = (rightVal > 0) ? rightVal : -rightVal;
}

void Pololu_Zumo_SensorLED(bool lightOn)
{
  GPIOWrite(POLOLU_ZUMO_SENSOR_LED, lightOn);     // lights in log. 1
}

void Pololu_Zumo_YellowLED(bool lightOn)
{
  GPIOWrite(POLOLU_ZUMO_YELLOW_LED, lightOn);     // lights in log. 1
}

bool Pololu_Zumo_ButtonState(void)
{
  return GPIORead(POLOLU_ZUMO_BUTTON_SW3);
}

