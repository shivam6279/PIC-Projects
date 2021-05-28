#ifndef _AHRS_H_
#define _AHRS_H_

#include "MPU6050.h"

#define FUSION_BETA	0.05f // 0.05f

#define ROLLOFFSET 0.0
#define PITCHOFFSET 0.0
#define HEADINGOFFSET 0.0

extern XYZ MultiplyVectorQuaternion(XYZ, float q[4]);
extern XYZ RotateVectorEuler(XYZ, float, float, float);
extern void QuaternionToEuler(float[], float*, float*, float*);

extern void MadgwickQuaternionUpdate(float*, XYZ, XYZ, XYZ, float);
extern void MadgwickQuaternionUpdateGyro(float*, XYZ, float);
extern void MadgwickQuaternionUpdateAcc(float*, XYZ, float);
extern float invSqrt(float);

extern float roll_offset, pitch_offset, heading_offset;

#endif
