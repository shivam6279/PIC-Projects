#ifndef _PWM_H_
#define _PWM_H_

#include "settings.h"

extern void pwm_init(float freq);
extern void write_pwm(int num, int val);

#ifndef board_v4
	extern void set_pwm(int num, int on, int off);
	extern void write_pwm_four(int num, int a, int b, int c, int d);
#endif

extern int pwm_max;
extern int motor_off, motor_max;

#endif