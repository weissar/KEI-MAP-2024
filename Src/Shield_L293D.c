/**
 * \brief Shield signals on "Arduino" pins
 */
// D0
// D1
// D2 - PA10 - JP3 - ??
// D3 - PB3 - PWM2B - motor 2 = PWM2/2 (AF01)
// D4 - PB5 - DIR_CLK - 74HC595 SCK
// D5 - PB4 - PWM0B - motor 3 = PWM3/1 (AF02)
// D6 - PB10 - PWM0A - motor 4 = PWM2/3 (AF01)
// D7 - PA8 - DIR_EN - 74HC595 G + PullUp
//
// D8 - PA9 - DIR_SER - 74HC595 SER
// D9 - PC7 - PWM1A - servo 2 = PWM3/2 (AF02)
// D10 - PB6 - PWM1B - servo 1 = PWM4/1 (AF02)
// D11 - PA7 - PWM2A - motor 1 = PWM1/1N (AF01), TIM3CH2 (AF02)
// D12 - PA6 - DIR_LATCH - 74HC595 RCK - Output register clock
// D13 - PA5

// QA - M3A
// QB - M2A
// QC - M1A
// QD - M1B
// QE - M2B
// QF - M4A
// QG - M3B
// QH - M4B

#include "Shield_L293D.h"

/**
 * \brief Control of '595 shift register
 *
 */
#define MOTORS_DIR_CLK    GPIOB,5
#define MOTORS_DIR_SER    GPIOA,9
#define MOTORS_DIR_EN     GPIOA,8
#define MOTORS_DIR_LATCH  GPIOA,6

/**
 * \brief Flag - is 595 settings finished ?
 */
static bool _m_595_ready = false;
/**
 * \brief Stored value for '595
 */
static uint8_t _m_595_value = 0;

/**
 * \fn void _motor_595_init_pins(void)
 * \brief Set GPIO for '595 serial output
 *
 */
static void _motor_595_init_pins(void)
{
  STM_SetPinGPIO(MOTORS_DIR_CLK, ioPortOutputPP);
  GPIOWrite(MOTORS_DIR_CLK, 0);
  STM_SetPinGPIO(MOTORS_DIR_LATCH, ioPortOutputPP);
  GPIOWrite(MOTORS_DIR_LATCH, 0);
  STM_SetPinGPIO(MOTORS_DIR_EN, ioPortOutputPP);
  GPIOWrite(MOTORS_DIR_EN, 0);        // permanent ON
  STM_SetPinGPIO(MOTORS_DIR_SER, ioPortOutputPP);

  _m_595_ready = true;    // init finished
}

/**
 * \fn bool _motor_595_set_motor(int, bool, bool)
 * \brief Set internal buffer for '595 outputs for motors
 * \remark !! only set data
 * \remark !! transfer to '595 must be executed at another place
 *
 * \param motor_id Selected motor - 1..4
 * \param in_a Signal IN_A at L293D
 * \param in_b Signal IN_B at L293D
 * \return Success if parameters are correct
 */
static bool _motor_595_set_motor(int motor_id, bool in_a, bool in_b)
{
  if (!_m_595_ready)
    _motor_595_init_pins();

  switch(motor_id)
  {
    case 1:
      _m_595_value &= ~(0x04u | 0x08u);
      if (in_a) _m_595_value |= 0x04u;
      if (in_b) _m_595_value |= 0x08u;
      break;
    case 2:
      _m_595_value &= ~(0x02u | 0x10u);
      if (in_a) _m_595_value |= 0x02u;
      if (in_b) _m_595_value |= 0x10u;
      break;
    case 3:
      _m_595_value &= ~(0x01u | 0x40u);
      if (in_a) _m_595_value |= 0x01u;
      if (in_b) _m_595_value |= 0x40u;
      break;
    case 4:
      _m_595_value &= ~(0x20u | 0x80u);
      if (in_a) _m_595_value |= 0x20u;
      if (in_b) _m_595_value |= 0x80u;
      break;
    default:
      return false;
  }

  return true;
}

/**
 * \fn bool _motor_595_set_stepper(int, uint8_t)
 * \brief Set internal byte for '595, 4 bits for 4 stepper signals
 *
 * \param stepper_id Select stepper - 1..2
 * \param nibble 4-bit combination, used lower nibble only
 * \return Success if parameters OK
 */
static bool _motor_595_set_stepper(int stepper_id, uint8_t nibble)
{
  if (!_m_595_ready)
    _motor_595_init_pins();

  nibble &= 0x0f;                 // for safe ...

  switch(stepper_id)
  {
    case 2:
      _m_595_value &= ~(0x01u | 0x40u | 0x20u | 0x80u);
      if (nibble & 0x01) _m_595_value |= 0x01u;     // m3a
      if (nibble & 0x02) _m_595_value |= 0x40u;     // m3b
      if (nibble & 0x04) _m_595_value |= 0x20u;     // m4a
      if (nibble & 0x08) _m_595_value |= 0x80u;     // m4b
      break;
    case 1:
      _m_595_value &= ~(0x04u | 0x08u | 0x02u | 0x10u);
      if (nibble & 0x01) _m_595_value |= 0x04u;     // m1a
      if (nibble & 0x02) _m_595_value |= 0x08u;     // m1b
      if (nibble & 0x04) _m_595_value |= 0x02u;     // m2a
      if (nibble & 0x08) _m_595_value |= 0x10u;     // m2b
      break;
    default:
      return false;
  }

  return true;
}

/**
 * \fn void _motor_595_send(void)
 * \brief Low-level function to send '595 data to device
 *
 */
static void _motor_595_send(void)
{
  if (!_m_595_ready)
    _motor_595_init_pins();

  uint8_t d = _m_595_value;       // get global value
  for(int i = 0; i < 8; i++)
  {
    GPIOWrite(MOTORS_DIR_SER, d & 0x80);     // MSB
    d <<= 1;

    GPIOWrite(MOTORS_DIR_CLK, 1);
    __NOP();
    GPIOWrite(MOTORS_DIR_CLK, 0);
  }

  GPIOWrite(MOTORS_DIR_LATCH, 1);
  __NOP();
  GPIOWrite(MOTORS_DIR_LATCH, 0);
}

/**
 * \brief Blocking variables - cannot use more devices on single signal
 */
static bool _servo1_enabled = false;
static bool _servo2_enabled = false;

static bool _motor1_enabled = false;
static bool _motor2_enabled = false;
static bool _motor3_enabled = false;
static bool _motor4_enabled = false;

static bool _stepper1_enabled = false;
static bool _stepper2_enabled = false;

/**
 * \fn bool motors_servo_enable(int)
 * \brief Initialize output and TIMx PWM for servos
 * \remark !! Available only servo 1 at TIM4
 * \remark !! Servo 2 collides with TIM3 output for motors
 *
 * \param servo Servo num - 1..2
 * \return Success if servo num is valid and not blocked
 */
bool motors_servo_enable(int servo)
{
  switch(servo)
  {
    case 1:
      STM_SetPinGPIO(GPIOB, 6, ioPortAlternatePP);
      STM_SetAFGPIO(GPIOB, 6, 2);

      {
        uint32_t apb1 = STM_GetTimerClock(4);

        if (!(RCC->APB1ENR & RCC_APB1ENR_TIM4EN))
        {
          RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
          RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST;
          RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM4RST;
        }

        TIM4->CR1 = 0;                  // count UP
        TIM4->CR1 |= TIM_CR1_ARPE;
        TIM4->CR2 = 0;

        TIM4->PSC = apb1 / 1E5 - 1;    // 10us = 100kHz
        TIM4->ARR = 5000 - 1;          // 5000 x 0.1ms = 50ms

        TIM4->CR1 |= TIM_CR1_CEN;

        TIM4->CCMR1 &= ~TIM_CCMR1_OC1M;
        TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

        TIM4->CCER |= TIM_CCER_CC1E;
        TIM4->CCR1 = 150;   // 1.5ms
      }

      _servo1_enabled = true;
      break;
    case 2:
      if (_motor1_enabled || _motor3_enabled)   // share TIM3
        return false;

      //TODO generate pulses manually via Timer-interrupt
      return false;         // eg. not implepmented
    default:
      return false;
  }

  return true;
}

/**
 * \fn bool motors_servo_usec(int, uint32_t)
 * \brief Set pulse width for servo
 * \remark !! only servo 1, internally calculate in 10us steps
 *
 * \param servo Servo num - 1..2
 * \param width_usec Pulse width [usec], normally 1000..2000, center = 1500
 * \return
 */
bool motors_servo_usec(int servo, uint32_t width_usec)
{
  switch(servo)
  {
    case 1:
      if (!_servo1_enabled)
        return false;

      width_usec /= 10;       // parameter in usec, CCR value in 10th usec
      if (width_usec > TIM4->ARR)
        return false;

      TIM4->CCR1 = width_usec;
      return true;
    case 2:
      if (!_servo1_enabled)
        return false;

      break;
  }

  return false;
}

/**
 * \fn bool _setTimerPWM(int, int, uint32_t)
 * \brief Internally sets TIMx for PWM generation
 *
 * \param timerNum Which timer - only 2, 3 and 4 is valid
 * \param channel Which channel 1..4
 * \param pwm_freq Required frequency, PSC/ARR calculated from ABPx clock
 * \return
 */
static bool _setTimerPWM(int timerNum, int channel, uint32_t pwm_freq)
{
  TIM_TypeDef *TIMx = 0;
  uint32_t maskEnr = 0, maskRst = 0;
  switch(timerNum)
  {
    case 2:
      TIMx = TIM2;
      maskEnr = RCC_APB1ENR_TIM2EN;
      maskRst = RCC_APB1RSTR_TIM2RST;
      break;
    case 3:
      TIMx = TIM3;
      maskEnr = RCC_APB1ENR_TIM3EN;
      maskRst = RCC_APB1RSTR_TIM3RST;
      break;
    case 4:
      TIMx = TIM4;
      maskEnr = RCC_APB1ENR_TIM4EN;
      maskRst = RCC_APB1RSTR_TIM4RST;
      break;
    default:
      return false;
  }

  if ((maskEnr == 0) || (maskRst == 0))
    return false;

  //TODO select APB1 or APB2 timersa
  if (!(RCC->APB1ENR & maskEnr))
  {
    RCC->APB1ENR |= maskEnr;
    RCC->APB1RSTR |= maskRst;
    RCC->APB1RSTR &= ~maskRst;
  }

  TIMx->CR1 = 0                  // count UP & stop
    // ARR not buffered ...  | TIM_CR1_ARPE;
    // there was problem on TIM2 - is 32b CNT !!
    ;
  TIMx->CR2 = 0;

  uint32_t apbClock = STM_GetTimerClock(timerNum);

  TIMx->PSC = apbClock / pwm_freq - 1;
  TIMx->ARR = 100 - 1;           // 100 steps

  TIMx->CR1 |= TIM_CR1_CEN;

  switch(channel)
  {
    case 1:
      TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;
      TIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

      TIMx->CCER |= TIM_CCER_CC1E;
      TIMx->CCR1 = 0;               // nothing
      break;
    case 2:
      TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;
      TIMx->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

      TIMx->CCER |= TIM_CCER_CC2E;
      TIMx->CCR2 = 0;
      break;
    case 3:
      TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;
      TIMx->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;

      TIMx->CCER |= TIM_CCER_CC3E;
      TIMx->CCR3 = 0;
      break;
    case 4:
      TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;
      TIMx->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

      TIMx->CCER |= TIM_CCER_CC4E;
      TIMx->CCR4 = 0;
      break;
  }

  //TODO special settings for TIM1

  return true;
}

/**
 * \fn bool motors_motor_enable(int, uint32_t)
 * \brief Enable motor, set PWM generation with frequency
 * \remark PWM in 100 steps
 *
 * \param motor Motor 1..4
 * \param pwm_freq Required PWM frequency
 * \return Success if parameters valid
 */
bool motors_motor_enable(int motor, uint32_t pwm_freq)
{
  if (!_m_595_ready)
    _motor_595_init_pins();

  switch(motor)
  {
    case 1:
      if (_stepper1_enabled || _servo2_enabled)   // same pins or TIM3
        return false;

      _motor_595_set_motor(1, false, false);      // OFF
      _motor_595_send();

      STM_SetPinGPIO(GPIOA, 7, ioPortAlternatePP);
      STM_SetAFGPIO(GPIOA, 7, 2);

      _motor1_enabled = _setTimerPWM(3, 2, pwm_freq);
      return _motor1_enabled;
    case 2:
      if (_stepper1_enabled)                      // same pins
        return false;

      _motor_595_set_motor(2, false, false);      // OFF
      _motor_595_send();

      STM_SetPinGPIO(GPIOB, 3, ioPortAlternatePP);
      STM_SetAFGPIO(GPIOB, 3, 1);

      _motor2_enabled = _setTimerPWM(2, 2, pwm_freq);
      return _motor2_enabled;
    case 3:
      if (_stepper2_enabled || _servo2_enabled)   // same pins or TIM3
        return false;

      _motor_595_set_motor(3, false, false);      // OFF
      _motor_595_send();

      STM_SetPinGPIO(GPIOB, 4, ioPortAlternatePP);
      STM_SetAFGPIO(GPIOB, 4, 2);

      return true == (_motor3_enabled = _setTimerPWM(3, 1, pwm_freq));
    case 4:
      if (_stepper2_enabled)              // same pins
        return false;

      _motor_595_set_motor(4, false, false);      // OFF
      _motor_595_send();

      STM_SetPinGPIO(GPIOB, 10, ioPortAlternatePP);
      STM_SetAFGPIO(GPIOB, 10, 1);

      return true == (_motor4_enabled = _setTimerPWM(2, 3, pwm_freq));
    default:
      return false;
  }
}

/**
 * \fn bool motors_motor_run(int, uint32_t, bool)
 * \brief Set PWN duty (= motor speed) and direction
 * \remark Now works only motor 1
 *
 * \param motor ID motor 1..4
 * \param width_perc
 * \param direction Left/right rotation, controlled via IN_A/IN_B signals
 * \return Success if selected motor enabled
 */
bool motors_motor_run(int motor, uint32_t width_perc, bool direction)
{
  if (width_perc > 100)
    width_perc = 100;

  volatile uint32_t *ptrCCRx = 0;

  switch(motor)
  {
    case 1:
      if (!_motor1_enabled)
        return false;

      ptrCCRx = &(TIM3->CCR2);
      break;
    case 2:
      if (!_motor2_enabled)
        return false;

      ptrCCRx = &(TIM2->CCR2);
      break;
    case 3:
      if (!_motor3_enabled)
        return false;

      ptrCCRx = &(TIM3->CCR1);
      break;
    case 4:
      if (!_motor4_enabled)
        return false;

      ptrCCRx = &(TIM2->CCR3);
      break;
    default:
      return false;
  }

  if (!ptrCCRx)             // not set ?
    return false;           // fail

  *ptrCCRx = 0;             // temporary stop

  _motor_595_set_motor(motor, direction, !direction);      // real directions must be checked
  _motor_595_send();

  *ptrCCRx = width_perc;

  return true;
}

/**
 * \fn bool motors_stepper_enable(int)
 * \brief Init and enable stepper
 * \remark In this moment works only stepper 2
 *
 * \param stepper Number - 1..2
 * \return Success if parameter OK
 */
bool motors_stepper_enable(int stepper)
{
  switch(stepper)
  {
    case 2:
      if (_motor3_enabled || _motor4_enabled)     // pin/connector collision
        return false;

      _motor_595_set_stepper(2, 0x00);      // OFF
      _motor_595_send();

      STM_SetPinGPIO(GPIOB, 4, ioPortOutputPP);     // motor 3 en
      GPIOWrite(GPIOB, 4, 1);               // activate
      STM_SetPinGPIO(GPIOB, 10, ioPortOutputPP);    // motor 4 en
      GPIOWrite(GPIOB, 10, 1);              // activate

      _stepper2_enabled = true;
      break;
    case 1:
      if (_motor3_enabled || _motor4_enabled)     // pin/connector collision
        return false;

      //TODO add pin init
      return false;         // now not implemented
    default:
      return false;
  }

  return true;
}

/**
 * \fn bool motors_stepper_set_bits(int, uint8_t)
 * \brief Set 4 signals for stepper
 *
 * \param stepper Num - 1..2
 * \param nibble Value - only lower nibble
 * \return Success if parameters OK
 */
bool motors_stepper_set_bits(int stepper, uint8_t nibble)
{
  switch(stepper)
  {
    case 2:
      if(!_stepper2_enabled)
        return false;

      bool bb = _motor_595_set_stepper(stepper, nibble);
      _motor_595_send();
      return bb;
    case 1:
    default:
      return false;
  }
}

