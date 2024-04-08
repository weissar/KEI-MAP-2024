#ifndef MAP_SHIELD_H_
#define MAP_SHIELD_H_

#include "stm_core.h"

#define RGB_BLUE    GPIOB,13
#define RGB_GREEN   GPIOB,14
#define RGB_RED     GPIOB,15

#define BTN_LEFT    GPIOB, 5
#define BTN_RIGHT   GPIOB, 4

void Init8LED(void);
void Write8LED(uint8_t val);
void InitSPILED(void);
void WriteSPILED(uint8_t val);

#endif /* MAP_SHIELD_H_ */
