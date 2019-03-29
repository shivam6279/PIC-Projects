#ifndef _AHRS_H_
#define _AHRS_H_

#include "10DOF.h"

#define FUSION_BETA	1.0f

#define ALTITUDE_KF_Q	0.1f
#define ALTITUDE_KF_R	0.1f

extern void MultiplyVectorQuarternion(float q[4], XYZ r, XYZ *v);
extern void RotateVector(float, float, float, XYZ*);
extern void QuaternionToEuler(float[], float*, float*, float*);
extern void MadgwickQuaternionUpdate(float*, XYZ, XYZ, XYZ, float);
extern float invSqrt(float);

extern void altitude_KF_reset();
extern void altitude_KF_propagate(float, float);
extern void altitude_KF_update(float, float);
extern float altitude_KF_getAltitude();
extern float altitude_KF_getVelocity();

extern float altitude_kf_P[2][2];
extern float altitude_kf_h;
extern float altitude_kf_v;

#endif