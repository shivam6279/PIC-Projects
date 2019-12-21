#ifndef _AHRS_H_
#define _AHRS_H_

#include "10DOF.h"

#define ACC_GRAVITY 9.8f //m/s^2

#define FUSION_BETA	1.0f

#define ALITUDE_KF_ACC_BUFFER_SIZE 10
#define ALTITUDE_KF_Q	0.2f
#define ALTITUDE_KF_R	0.1f

extern void MultiplyVectorQuarternion(float q[4], XYZ, XYZ*);
extern void RotateVector(float, float, float, XYZ*);
extern void QuaternionToEuler(float[], float*, float*, float*);
extern void MadgwickQuaternionUpdate(float*, XYZ, XYZ, XYZ, float);
extern float invSqrt(float);

extern void GetCompensatedAcc(float q[4], float, XYZ, XYZ*, XYZ*);

extern void altitude_KF_reset();
extern void altitude_KF_propagate(float, float);
extern void altitude_KF_update(float);
extern float altitude_KF_getAltitude();
extern float altitude_KF_getVelocity();

extern float roll_offset, pitch_offset, heading_offset;

#endif
