#ifndef _10DOF_H_
#define _10DOF_H_

#include "settings.h"
#include <stdbool.h>

#define ACC_GRAVITY 9.81f //m/s^2

#define ACC_X_OFFSET 0.0f
#define ACC_Y_OFFSET 0.0f
#define ACC_Z_OFFSET 0.0f

#define ACC_X_GAIN 1.0f
#define ACC_Y_GAIN 1.0f
#define ACC_Z_GAIN 1.0f

#define GYRO_X_OFFSET 120.0f
#define GYRO_Y_OFFSET -20.0f
#define GYRO_Z_OFFSET -150.0f

#define GYRO_X_GAIN 32.5f
#define GYRO_Y_GAIN 32.5f
#define GYRO_Z_GAIN 32.5f

#define COMPASS_X_MIN -800.0f
#define COMPASS_X_MAX 1100.0f

#define COMPASS_Y_MIN -400.0f
#define COMPASS_Y_MAX 1900.0f

#define COMPASS_Z_MIN -1200.0f
#define COMPASS_Z_MAX 1200.0f

#define ACC_LPF 0.5
#define GYRO_LPF 0.5
#define COMPASS_LPF 0.5

typedef struct {
    float x;
    float y;
    float z;
} XYZ;

extern void VectorReset(XYZ *v);
extern XYZ VectorAdd(XYZ, XYZ);
extern XYZ VectorSubtract(XYZ, XYZ);
extern XYZ VectorScale(XYZ, float);

//MPU6050
#define MPU6050_ADDR 0x68

extern void MPU6050_Init();
extern bool GetRawAcc(XYZ*);
extern bool GetAcc(XYZ*);
extern bool GetRawGyro(XYZ*);
extern bool GetGyro(XYZ*);

//Magnetometer

#define LIS3MDL_ADDR 0x1C

void LIS3MDL_Init();

extern void ComputeCompassOffsetGain(XYZ, XYZ);
extern bool GetRawCompass(XYZ*);
extern bool GetCompass(XYZ*);

//Pressure sensors

extern unsigned char oversampling_delay;

//BMP390

#define BMP390_ADDR 0x77

#define OVERSAMPLING 3  //0 - 3
#define SEA_LEVEL_PRESSURE 101325UL

extern void BMP180_Init();
extern void StartPressureRead();
extern void ReadRawPressure();
extern void StartTemperatureRead();
extern void ReadRawTemperature();
extern double ComputeTemperature();
extern float GetAltitude();

extern XYZ acc_offset, acc_gain;
extern XYZ gyro_offset, gyro_gain;
extern XYZ compass_offset, compass_gain;
extern unsigned int MS5611_fc[6];

#endif
