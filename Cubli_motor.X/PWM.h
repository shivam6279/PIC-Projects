#ifndef _PWM_H_
#define _PWM_H_

#include <stdbool.h>

extern void PwmInit(float freq);
extern void WritePwm(int num, int val);

extern bool U_bemf();
extern bool V_bemf();
extern bool W_bemf();

extern volatile unsigned char comparator;
extern unsigned int PWM_MAX;

#endif