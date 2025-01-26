#ifndef _PWM_H_
#define _PWM_H_

#include <stdbool.h>

extern void PwmInit(float freq);
extern void WritePwm(int num, int val);

extern unsigned int PWM_MAX;

#endif
