#include "pololu_qtr.h"

/**
 * \def BB_REG, BB_RAM
 * \brief Simple access to bits via bitbanding
 *
 */
#ifndef BB_REG
#define BB_REG(reg, bit) (*(uint32_t *)(PERIPH_BB_BASE + ((uint32_t)(&(reg)) - PERIPH_BASE) * 32 + 4 * (bit)))
#define BB_RAM(adr, bit) (*(uint32_t *)(SRAM_BB_BASE + ((uint32_t)(adr) - SRAM_BASE) * 32 + 4 * (bit)))
#endif

/**
 * \def NULL
 * \brief Represents "invalid pointer value" - address zero
 *
 */
#ifndef NULL
#define NULL  ((void *)0)
#endif

/**
 * \struct gpio_pin_struct
 * \brief Pair GPIOx pointer and number of pin
 * \remark Used for definition pins with connected QTR sensors
 *
 */
typedef struct gpio_pin_struct
{
    GPIO_TypeDef *gpio;
    int32_t pin;
} gpio_pin;

/**
 * \def MAX_NUM_SENZORS
 * \brief Real sensor count must be <= predefined MAX
 *
 */
#define MAX_NUM_SENZORS   10

/**
 * \def senzors, senzorsFinal
 * \brief Meassured data are stored to "sensors", after last on there is copied to "Final"
 * \remark Simulates double-buffer, data for calculations are stable (in "Final")
 *
 */
static volatile uint16_t senzors[MAX_NUM_SENZORS];
static volatile uint16_t senzorsFinal[MAX_NUM_SENZORS];

/**
 * \def sensMax, sensMin
 * \brief Max/min values create via calibration
 * \remark Used for calculation "equalized" value
 * \remark For every sensor
 *
 */
static uint16_t sensMax[MAX_NUM_SENZORS];
static uint16_t sensMin[MAX_NUM_SENZORS];

/**
 * \def fSendK, uSensD
 * \brief Precalculated values for "normalisation" - expand raw data to 0..1000
 * \remark For every sensor
 *
 */
static float fSensK[MAX_NUM_SENZORS];
static uint16_t uSensO[MAX_NUM_SENZORS];

static volatile bool meassureProgress = false;    // meass. progress flag
static volatile bool meassureEnable = false;      // flag to enable meass.
static volatile int cntMeassureStart = 0;         // internal counters
static volatile int cntMeassurePulse = 0;

/**
 * \def _pins, _pinCount
 * \brief List of pins with sensors connected
 * \remark Initialized in "constructor"
 * \remark Pin Count must be <= MAX sensors count
 */
static volatile gpio_pin *_pins = NULL;
static volatile int _pinCount = 0;

static bool hispeedClock = false;         // flag with detection 16 or 100MHz HCLK
static uint32_t maxMeassInterval = 4000;  // used to calcuulate period eq 20ms

//TODO make configurable ... bool QTR_InitSensors(volatile gpio_pin *pins, int count)
/**
 * \fn bool Pololu_QTR_InitSensors(eQTRDevices)
 * \brief Init all HW parts - GPIO, TIM, IRQ, init internal data arrays
 * \remark !! uses TIM5 with IRQ, for meassure duration of pulses
 *
 * \param deviceType - predefined type of HW module with QTR sensors
 * \return Success
 */
bool Pololu_QTR_InitSensors(eQTRDevices deviceType)
{
  switch(deviceType)
  {
    case QTR_Device_Zumo:
      // D4, A3, D11, A0, A2, D5
      #define ZUMO_SENSOR_U1    GPIOB,5
      #define ZUMO_SENSOR_U2    GPIOB,0
      #define ZUMO_SENSOR_U3    GPIOA,7
      #define ZUMO_SENSOR_U4    GPIOA,0
      #define ZUMO_SENSOR_U5    GPIOA,4
      #define ZUMO_SENSOR_U6    GPIOB,4

      #define ZUMO_SENSOR_COUNT 6

      static const gpio_pin pins_zumo[ZUMO_SENSOR_COUNT] =
      {
        { ZUMO_SENSOR_U1 },
        { ZUMO_SENSOR_U2 },
        { ZUMO_SENSOR_U3 },
        { ZUMO_SENSOR_U4 },
        { ZUMO_SENSOR_U5 },
        { ZUMO_SENSOR_U6 },
      };

      _pins = (volatile gpio_pin *)pins_zumo;     // remove warnig
      _pinCount = ZUMO_SENSOR_COUNT;

      break;
    default:
      return false;         // fail - uknown device
  }

  for(int i = 0; i < _pinCount; i++)
  {
    STM_SetPinGPIO(_pins[i].gpio, _pins[i].pin, ioPortOutputPP);
    GPIOWrite(_pins[i].gpio, _pins[i].pin, true);

    senzors[i] = 0;
    senzorsFinal[i] = 0;
  }

  Pololu_QTR_ClearCalibrate();

  // use TIM5 for timing
  if (!(RCC->APB1ENR & RCC_APB1ENR_TIM5EN))
  {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM5RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM5RST;
  }

  TIM5->CR1 = 0
      | TIM_CR1_DIR       // down
      ;
  TIM5->CR2 = 0;

  TIM5->PSC = STM_GetTimerClock(5) / 1E6 - 1;       // 1us

  if (SystemCoreClock > 16E6)
  {
    hispeedClock = true;
    TIM5->ARR = 10 - 1;  // N x = N us
  }
  else
  {
    TIM5->ARR = 50 - 1;  // N x = N us
  }

  TIM5->DIER = TIM_DIER_UIE;
  NVIC_EnableIRQ(TIM5_IRQn);

  TIM5->CR1 |= TIM_CR1_CEN;

  maxMeassInterval = 20000 / TIM5->ARR;       // 20ms in ARR steps

  return true;                        // success
}

/**
 * \fn bool Pololu_QTR_ClearCalibrate(void)
 * \brief Clears calibration min/max vaues
 *
 * \return Success, now evertime OK
 */
bool Pololu_QTR_ClearCalibrate(void)
{
  if (_pinCount == 0)
    return false;

  for(int i = 0; i < _pinCount; i++)
  {
    sensMax[i] = 0;
    sensMin[i] = UINT16_MAX;
    uSensO[i] = 0;
    fSensK[i] = 1;
  }

  return true;
}

/**
 * \fn bool Pololu_QTR_StepCalibrate(void)
 * \brief Single step of calibration, search MIN and MAX value of all sensors
 * \remark Recalculates coeficients for normalisation of RAW values
 *
 * \return Success, now evertime OK
 */
bool Pololu_QTR_StepCalibrate(void)
{
  if (_pinCount == 0)
    return false;

  for(int i = 0; i < _pinCount; i++)
  {
    if (sensMax[i] < senzorsFinal[i])
      sensMax[i] = senzorsFinal[i];
    if (sensMin[i] > senzorsFinal[i])
      sensMin[i] = senzorsFinal[i];
  }

  for(int i = 0; i < _pinCount; i++)
  {
    uSensO[i] = sensMin[i];
    fSensK[i] = ((float)1000) / (sensMax[i] - sensMin[i]);
  }

  return true;
}

/**
 * \fn uint16_t Pololu_QTR_LineDetect(void)
 * \brief Calculation of single value which represents position on the line
 *
 * \return Value between 0 .. 1000x _numSensors
 */
uint16_t Pololu_QTR_LineDetect(void)
{
  uint32_t _sumW = 0;
  uint32_t _sum = 0;
  uint32_t _moc = 0;
  bool _lineDetected = false;       // tyto 2 veci inspirovane QTR lib pro Arduino
  static uint16_t last_val = 0;

  if (_pinCount == 0)
    return 0;

  for(int i = 0; i < _pinCount; i++)
  {
    uint16_t val = (uint16_t)((senzorsFinal[i] - uSensO[i]) * fSensK[i]);

    if (val > 200)
      _lineDetected = true;  // aspon nekde je trochu cerna

    if (val > 50)
    {
      _sumW += _moc * val;
      _sum += val;
    }

    _moc += 1000;
  }

  if (_lineDetected)
  {
    last_val = _sumW / _sum;
  }
  else
  {
    if (last_val < ((_pinCount - 1) * 1000 / 2))
      last_val = 0;
    else
      last_val = (_pinCount - 1) * 1000;
  }

  return last_val;
}

/**
 * \fn bool Pololu_QTR_FillSensors(uint16_t*)
 * \brief Generate normalized values of sensors - 0..1000 for every
 *
 * \param pSens Array for N 16-bit values (normalised)
 * \return Success, fail if sensors not initialized
 *
 */
bool Pololu_QTR_FillSensors(uint16_t *pSens)
{
  if (_pinCount == 0)
    return false;

  for(int i =0; i < _pinCount; i++)
    pSens[i] = (uint16_t)((senzorsFinal[i] - uSensO[i]) * fSensK[i]);

  return true;
}

/**
 * \fn bool Pololu_QTR_FillSensRaw(uint16_t*)
 * \brief Fill raw values to passed array
 *
 * \param pSens Array for N 16-bit values (raw)
 * \return Success, fail if sensors not initialized
 */
bool Pololu_QTR_FillSensRaw(uint16_t *pSens)
{
  if (_pinCount == 0)
    return false;

  for(int i =0; i < _pinCount; i++)
    pSens[i] = senzorsFinal[i];

  return true;
}

/**
 * \fn void Pololu_QTR_EnableMeassure(bool)
 * \brief Enable/disable meassurement
 * \remark Default is disabled
 * \remark Controls activity in TIMx-IRQ
 *
 * \param en True for enable meassuring
 */
void Pololu_QTR_EnableMeassure(bool en)
{
  meassureEnable = en;
}

/**
 * \fn void TIM5_IRQHandler(void)
 * \brief Timer handler to meassure pulse width
 * \remark In QTR sensors it corresponds intensity of reflected light
 *
 */
void TIM5_IRQHandler(void)
{
  if (TIM5->SR & TIM_SR_UIF)
  {
    BB_REG(TIM5->SR, 0) = 0;        // clear UIF

    if (meassureEnable)
    {
      if (meassureProgress)         // time meassure in progress ?
      {
        if (cntMeassurePulse >= maxMeassInterval) // finished in 20ms
        {
          for(int i = 0; i < _pinCount; i++)
            senzorsFinal[i] = senzors[i];    // double buffer copying

          meassureProgress = false;   // flag for finish
          cntMeassureStart = 0;       // start next time
        }
        else                          // not finished
        {
          cntMeassurePulse++;         // meassure max. interval

          for(int i = 0; i < _pinCount; i++)  // for all sensors
          {
            // if (GPIORead(_pins[i].gpio, _pins[i].pin))
            if (BB_REG(_pins[i].gpio->IDR, _pins[i].pin))   // bitband is faster
              senzors[i]++;           // increment when log 1 is readed
          }
        }
      }
      else                          // no meassure, entry pusle is generated
      {
        if (cntMeassureStart == 0)  // beginning of emass. sequence ?
        {
          for(int i = 0; i < _pinCount; i++)  // for all pins
          {                         // set GPIO as OUTPUT - 01
            _pins[i].gpio->MODER |= 0x00000001 << (2 * _pins[i].pin);
          }

          for(int i = 0; i < _pinCount; i++)  // for all pins
          {
            // GPIOWrite(_pins[i].gpio, _pins[i].pin, true);
            BB_REG(_pins[i].gpio->ODR, _pins[i].pin) = 1;   // set output to 1

            senzors[i] = 0;         // clear counters
          }
        }

        if (cntMeassureStart >= 2)      // min 2x Xus = required > 10us
        {
          for(int i = 0; i < _pinCount; i++)
          {                         // set GPIO to "input float" - MODER = 00
            _pins[i].gpio->MODER &= ~(0x00000003 << (2 * _pins[i].pin));
          }

          cntMeassurePulse = 0;     // flag - pulse finished
          meassureProgress = true;  // start time meassurement
        }
        else                        // entry pulse in prgrress
        {
          cntMeassureStart++;       // meassure pulse
        }
      }
    }
    else                            // meassure disabled
    {
      cntMeassureStart = 0;         // set to init values of sequnce
      meassureProgress = false;
    }
  }
}

