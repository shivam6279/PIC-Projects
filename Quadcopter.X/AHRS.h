#ifndef _AHRS_H_
#define _AHRS_H_

#include "10DOF.h"

#define FUSION_BETA	0.15f//0.15f

#define ALITUDE_KF_ACC_BUFFER_SIZE 10
#define ALTITUDE_KF_Q	0.2f
#define ALTITUDE_KF_R	0.1f

extern XYZ MultiplyVectorQuaternion(XYZ, float q[4]);
extern XYZ RotateVectorEuler(XYZ, float, float, float);
extern void QuaternionToEuler(float[], float*, float*, float*);

extern XYZ GetCompensatedAcc(float[], XYZ, float);

extern void MadgwickQuaternionUpdate(float*, XYZ, XYZ, XYZ, float);
extern void MadgwickQuaternionUpdateGyro(float*, XYZ, float);
extern void MadgwickQuaternionUpdateAcc(float*, XYZ, float);
extern float invSqrt(float);

extern void altitude_KF_reset();
extern void altitude_KF_propagate(float, float);
extern void altitude_KF_update(float);
extern float altitude_KF_getAltitude();
extern float altitude_KF_setAltitude(float);
extern float altitude_KF_setVelocity(float);
extern float altitude_KF_getVelocity();

extern float roll_offset, pitch_offset, heading_offset;

#endif
