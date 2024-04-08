/*
 * stm_core.h
 *
 *  Created on: 17. 2. 2023
 *      Author: weissar
 */

#ifndef STM_CORE_H_
#define STM_CORE_H_

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  ioPortOutputPP,       // output Push-Pull type
  ioPortOutputOC,       // output Open Drain (like OC) type
  ioPortAnalog,         // analog input – for A/D
  ioPortInputFloat,     // input without pull-up/down
  ioPortInputPU,        // input with pull-up
  ioPortInputPD,        // input with pull-down
  ioPortAlternatePP,    // alternative function output with push-pull mode
  ioPortAlternateOC     // alternative function output - open drain
} eIoPortModes;

bool STM_SetPinGPIO(GPIO_TypeDef *pgpio, uint32_t bitnum, eIoPortModes mode);
bool STM_SetAFGPIO(GPIO_TypeDef *pgpio, uint32_t bitnum, uint32_t afValue);

void GPIOToggle(GPIO_TypeDef *pgpio, uint32_t bitnum);
bool GPIORead(GPIO_TypeDef *pgpio, uint32_t bitnum);
void GPIOWrite(GPIO_TypeDef *pgpio, uint32_t bitnum, bool state);

#define BOARD_BTN_BLUE  GPIOC,13
#define BOARD_LED       GPIOA,5

#endif /* STM_CORE_H_ */
