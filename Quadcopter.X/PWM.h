#ifndef _PWM_H_
#define _PWM_H_

#include "settings.h"

extern void pwm_init(float freq);
extern void write_pwm(int num, int val);

#ifndef board_v4
	extern void set_pwm(int num, int on, int off);
#endif

extern int PWM_MAX;
extern int MOTOR_OFF, MOTOR_MAX;

#endif