#ifndef _PWM_H_
#define _PWM_H_

#include "settings.h"

extern void pwm_init(float freq);
extern void write_pwm(int num, int val);

extern unsigned int PWM_MAX;
extern unsigned int MOTOR_OFF, MOTOR_MAX;

#endif