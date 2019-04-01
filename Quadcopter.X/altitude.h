#ifndef _altitude_H_
#define _altitude_H_

#define ALTITUDE_BUFFER_SIZE 3

extern void LoopAltitude(unsigned char*, unsigned long int*, unsigned long int*, double[], double, double*);
extern double GetTakeoffAltitude(double*);
#endif