#ifndef SHIELD_L293D_H_
#define SHIELD_L293D_H_

#include "stm_core.h"

bool motors_servo_enable(int servo);
bool motors_servo_usec(int servo, uint32_t width_usec);
bool motors_motor_enable(int motor, uint32_t pwm_freq);
bool motors_motor_run(int motor, uint32_t width_perc, bool direction);
bool motors_stepper_enable(int stepper);
bool motors_stepper_set_bits(int stepper, uint8_t nibble);

#endif /* SHIELD_L293D_H_ */
