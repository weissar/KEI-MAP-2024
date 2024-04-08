#include "stm_core.h"

/**
 * \fn void GPIOWrite(GPIO_TypeDef*, uint32_t, bool)
 * \brief
 *
 * \param pgpio
 * \param bitnum
 * \param state
 */
void GPIOWrite(GPIO_TypeDef *pgpio, uint32_t bitnum, bool state)
{
  if (state)
    pgpio->ODR |= 1 << bitnum;
  else
    pgpio->ODR &= ~(1 << bitnum);
}

/**
 * \fn void GPIOToggle(GPIO_TypeDef*, uint32_t)
 * \brief
 *
 * \param pgpio
 * \param bitnum
 */
void GPIOToggle(GPIO_TypeDef *pgpio, uint32_t bitnum)
{
  pgpio->ODR ^= 1 << bitnum;
}

/**
 * \fn bool GPIORead(GPIO_TypeDef*, uint32_t)
 * \brief
 *
 * \param pgpio
 * \param bitnum
 * \return
 */
bool GPIORead(GPIO_TypeDef *pgpio, uint32_t bitnum)
{
  return (pgpio->IDR & (1 << bitnum)) != 0;
}

/**
 * \fn bool STM_SetPinGPIO(GPIO_TypeDef*, uint32_t, eIoPortModes)
 * \brief
 *
 * \param pgpio Pointer to GPIO registers structure
 * \param bitnum Pin number - 0..15
 * \param mode Defined as enum
 * \return True if success (valid parameters)
 */
bool STM_SetPinGPIO(GPIO_TypeDef *pgpio, uint32_t bitnum, eIoPortModes mode)
{
  uint32_t enr_mask = 0;     // value to xxENR register
  uint32_t rstr_mask = 0;    // value to xxRSTR register

  switch((uint32_t)pgpio)    // detection which GPIO
  {
    case (uint32_t)GPIOA:
      enr_mask = RCC_AHB1ENR_GPIOAEN;
      rstr_mask = RCC_AHB1RSTR_GPIOARST;
      break;
    case (uint32_t)GPIOB:
      enr_mask = RCC_AHB1ENR_GPIOBEN;
      rstr_mask = RCC_AHB1RSTR_GPIOBRST;
      break;
    case (uint32_t)GPIOC:
      enr_mask = RCC_AHB1ENR_GPIOCEN;
      rstr_mask = RCC_AHB1RSTR_GPIOCRST;
      break;
    case (uint32_t)GPIOD:
      enr_mask = RCC_AHB1ENR_GPIODEN;
      rstr_mask = RCC_AHB1RSTR_GPIODRST;
      break;
    case (uint32_t)GPIOE:
      enr_mask = RCC_AHB1ENR_GPIOEEN;
      rstr_mask = RCC_AHB1RSTR_GPIOERST;
      break;
#if defined(STM32F411xE)  // ... add other types
#else
    case (uint32_t)GPIOF:
      enr_mask = RCC_AHB1ENR_GPIOFEN;
      rstr_mask = RCC_AHB1RSTR_GPIOFRST;
      break;
    case (uint32_t)GPIOG:
      enr_mask = RCC_AHB1ENR_GPIOGEN;
      rstr_mask = RCC_AHB1RSTR_GPIOGRST;
      break;
#endif
    case (uint32_t)GPIOH:
      enr_mask = RCC_AHB1ENR_GPIOHEN;
      rstr_mask = RCC_AHB1RSTR_GPIOHRST;
      break;
  }

  if ((enr_mask == 0) || (rstr_mask == 0))    // not selected GPIO ?
    return false;                             // return fail

  if (!(RCC->AHB1ENR & enr_mask))   // init selected - clock, ...
  {
    RCC->AHB1ENR |= enr_mask;       // enable peripheral clock
    RCC->AHB1RSTR |= rstr_mask;     // reset peripheral block
    RCC->AHB1RSTR &= ~rstr_mask;    // reset finish
  }

  pgpio->MODER &= ~(0x03 << (2 * bitnum));   // clear 2 bits in register
  pgpio->PUPDR &= ~(0x03 << (2 * bitnum));
  pgpio->OSPEEDR &= ~(0x03 << (2 * bitnum));

  switch(mode)
  {
    case ioPortOutputOC:
    case ioPortOutputPP:
      pgpio->MODER |= 0x01 << (2 * bitnum);     // 01 = output
      pgpio->OSPEEDR |= 0x03 << (2 * bitnum);   // 11 = high speed
      pgpio->PUPDR &= ~(0x03 << (2 * bitnum));  // 00 = no pu/pd

      if (mode == ioPortOutputOC)               // open collector (drain) ?
        pgpio->OTYPER |= 0x01 << bitnum;        // 1 = OC/Open drain
      else
        pgpio->OTYPER &= ~(0x01 << bitnum);     // 0 = push-pull
      break;
    case ioPortInputPU:                         // bits 00 = input
      pgpio->PUPDR &= ~(0x03 << (2 * bitnum));  // clear bits
      pgpio->PUPDR |= 0x01 << (2 * bitnum);     // 01 = pull-up
      break;
    case ioPortInputPD:
      pgpio->PUPDR &= ~(0x03 << (2 * bitnum));  // clear bits
      pgpio->PUPDR |= 0x02 << (2 * bitnum);     // 10 = pull-down
      break;
    case ioPortInputFloat:     // 00 = input mode, nothing to do
      pgpio->PUPDR &= ~(0x03 << (2 * bitnum)); // 00 = no pull-up/dn
      break;
    case ioPortAnalog:                          // 11 - analog mode
      pgpio->MODER |= 0x03 << (2 * bitnum);     // set bits
      break;
    case ioPortAlternatePP:
    case ioPortAlternateOC:
      pgpio->MODER |= 0x02 << (2 * bitnum);     // set bits

      if (mode == ioPortAlternateOC)
        pgpio->OTYPER |= 0x01 << bitnum;        // 1 = open-drain
      else
        pgpio->OTYPER &= ~(0x01 << bitnum);     // 0 = push-pull

      pgpio->OSPEEDR |= 0x03 << (2 * bitnum);   // high-speed = 11
      pgpio->PUPDR &= ~(0x03 << (2 * bitnum));  // 00 = no pull-up/pull-down
                              // don't forget set AFR registers !!!
      break;
    default:                                    // unknown mode ?
      return false;                             // return as fail
  }

  return true;                                  // return OK
}

/**
 * \fn bool STM_SetAFGPIO(GPIO_TypeDef*, uint32_t, uint32_t)
 * \brief
 *
 * \param pgpio Pointer to GPIO registers structure
 * \param bitnum Pin number - 0..15
 * \param afValue Alternate fanction num. for pin - see DS
 * \return True if success (valid parameters)
 */
bool STM_SetAFGPIO(GPIO_TypeDef *pgpio, uint32_t bitnum, uint32_t afValue)
{
  pgpio->AFR[(bitnum < 8) ? 0 : 1] &= ~(0x0f << (4 * (bitnum & 0x07)));              // vynuluj AF bity
  pgpio->AFR[(bitnum < 8) ? 0 : 1] |= ((afValue & 0x0f) << (4 * (bitnum & 0x07)));   // nastav AF bity
  return true;
}
