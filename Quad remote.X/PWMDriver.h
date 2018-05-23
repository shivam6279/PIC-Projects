#ifndef _PWMDriver_H_
#define _PWMDriver_H_

extern void pwm_driver_init(float);
extern void set_pwm(int, int, int);
extern void write_pwm(int, int);

#endif