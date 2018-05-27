#ifndef _altitude_H_
#define _altitude_H_

#include "PID.h"

extern void LoopAltitude(unsigned char*, unsigned long int*, unsigned long int*, double[], PID*, double, double*);
extern double GetTakeoffAltitude(double*);
#endif