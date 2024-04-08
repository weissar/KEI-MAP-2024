/*
 * ultrasonic_driver.h
 *
 *  Created on: Aug 19, 2023
 *      Author: Weissar
 */

#ifndef ULTRASONIC_DRIVER_H_
#define ULTRASONIC_DRIVER_H_

#include "stm_core.h"
#include "stm_core_addon.h"

// here for info !!
#define SHIELD_SONAR_TRIG GPIOA,1       // AD1 - generate 40us pulse
#define SHIELD_SONAR_ECHO GPIOA,0       // AD0 - read pulse width

uint32_t US_MeassureBlocking(void);
float US_ConvertToMM(uint32_t width_ms);

#endif /* ULTRASONIC_DRIVER_H_ */
