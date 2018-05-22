#ifndef _AHRS_H_
#define _AHRS_H_

#include "10DOF.h"

#define beta 1.0

extern void MultiplyVectorQuarternion(float q[4], XYZ r, XYZ *v);
extern void MadgwickQuaternionUpdate(float*, XYZ, XYZ, XYZ, float);
extern void MatrixInit(float *a);

#endif