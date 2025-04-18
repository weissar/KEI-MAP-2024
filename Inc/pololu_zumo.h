#ifndef _POLOLU_ZUMO_H_
#define _POLOLU_ZUMO_H_

#include "stm_core_addon.h"       // including stm_core.h too

bool Pololu_Zumo_InitMotors(void);
void Pololu_Zumo_Motors(int leftVal, int rightVal);

void Pololu_Zumo_SensorLED(bool lightOn);
void Pololu_Zumo_YellowLED(bool lightOn);

bool Pololu_Zumo_ButtonState(void);

#endif /* _POLOLU_ZUMO_H_ */
