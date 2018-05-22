#ifndef _PWMDriver_H_
#define _PWMDriver_H_

extern void PwmDriverInit(float freq);
extern void set_pwm(int num, int on, int off);
extern void write_pwm(int num, int val);
extern void write_pwm_four(int num, int a, int b, int c, int d);

#endif