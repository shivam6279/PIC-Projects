#ifndef _10DOF_H_
#define _10DOF_H_

#include "settings.h"
#include <stdbool.h>

#define ACC_GRAVITY 9.81f //m/s^2

#define ACC_X_OFFSET 0.0f
#define ACC_Y_OFFSET 0.0f
#define ACC_Z_OFFSET 0.0f

#define ACC_X_GAIN 8.0f / 32768.0f // 8g/15bits
#define ACC_Y_GAIN 8.0f / 32768.0f // 8g/15bits
#define ACC_Z_GAIN 8.0f / 32768.0f // 8g/15bits

#define GYRO_X_OFFSET 65.0f
#define GYRO_Y_OFFSET 136.0f
#define GYRO_Z_OFFSET -10.0f

#define GYRO_X_GAIN 2000.0f / 32768.0f // 2000 degrees per seconds/15bits
#define GYRO_Y_GAIN 2000.0f / 32768.0f // 2000 degrees per seconds/15bits
#define GYRO_Z_GAIN 2000.0f / 32768.0f // 2000 degrees per seconds/15bits

#define COMPASS_X_MIN -589.0f
#define COMPASS_X_MAX 821.0f

#define COMPASS_Y_MIN -687.0f
#define COMPASS_Y_MAX 617.0f

#define COMPASS_Z_MIN 492.0f
#define COMPASS_Z_MAX 1830.0f

typedef struct {
    float x;
    float y;
    float z;
} XYZ;

extern bool Init10DOF();

//Acceleromter
extern bool GetRawAcc(XYZ*);
extern bool GetAcc(XYZ*);

//Gyro
extern bool GetRawGyro(XYZ*);
extern bool GetGyro(XYZ*);

//Magnetometer
extern void ComputeCompassOffsetGain(XYZ, XYZ);
extern bool GetRawCompass(XYZ*);
extern bool GetCompass(XYZ*);

//Altimeter
extern float PressureToAltitude(float);
extern float GetAltitude();

extern XYZ acc_offset, acc_gain;
extern XYZ gyro_offset, gyro_gain;
extern XYZ compass_offset, compass_gain;

// Vector math
extern void VectorReset(XYZ *v);
extern XYZ VectorAdd(XYZ, XYZ);
extern XYZ VectorSubtract(XYZ, XYZ);
extern XYZ VectorScale(XYZ, float);
extern XYZ VectorTransform(XYZ, float[3][3]);

#endif
