#ifndef _10DOF_H_
#define _10DOF_H_

#include "settings.h"
#include <stdbool.h>

#define IMU_BUFFER_SIZE 10

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

#define GYRO_X_GAIN 98.0f
#define GYRO_Y_GAIN 98.0f
#define GYRO_Z_GAIN 106.0f

#define COMPASS_X_MIN -4255.0f
#define COMPASS_X_MAX 5930.0f

#define COMPASS_Y_MIN -5655.0f
#define COMPASS_Y_MAX 4126.0f

#define COMPASS_Z_MIN -5655.0f
#define COMPASS_Z_MAX 5406.0f

#ifdef BMP180
#define OVERSAMPLING 3  //0 - 3
#define SEA_LEVEL_PRESSURE 101325UL
#endif

#ifdef MS5611
#define OVERSAMPLING 4  //0 - 4
#define COMPENSATION 0
#endif

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

extern void MPU6050Init();
extern bool GetRawAcc(XYZ*);
extern bool GetAcc(XYZ*);
extern bool GetRawGyro(XYZ*);
extern bool GetGyro(XYZ*);

//Magnetometer

//HMC5883
#define HMC5883_ADDR 0x1E

#ifdef HMC5883
void HMC5883Init();
#endif

//QMC5883

#define QMC5883L_ADDR 0x0D

#ifdef QMC5883
void QMC5883Init();
bool QMC5883DataRdy();
#endif

extern void ComputeCompassOffsetGain(XYZ, XYZ);
extern bool GetRawCompass(XYZ*);
extern bool GetCompass(XYZ*);

//Pressure sensors

extern unsigned char oversampling_delay;

//BMP180

#define BMP180_ADDR 0x77

#ifdef BMP180
extern void BMP180Init();
extern void StartPressureRead();
extern void ReadRawPressure();
extern void StartTemperatureRead();
extern void ReadRawTemperature();
extern double ComputeTemperature();
extern float GetAltitude();
#endif

//MS5611

#define MS5611_ADDR 0x77

#ifdef MS5611
extern void MS5611Init();
extern void StartPressureRead();
extern unsigned long int ReadRawPressure();
extern void StartTemperatureRead();
extern unsigned long int ReadRawTemperature();
extern long int ComputePressure(unsigned long int, unsigned long int);
extern double ComputeTemperature(unsigned long int);
extern double GetAltitude(unsigned long int);
#endif

extern XYZ acc_offset, acc_gain;
extern XYZ gyro_offset;
extern XYZ compass_offset, compass_gain;
extern unsigned int MS5611_fc[6];

#if IMU_BUFFER_SIZE > 0
extern XYZ acc_buffer[IMU_BUFFER_SIZE], gyro_buffer[IMU_BUFFER_SIZE], compass_buffer[IMU_BUFFER_SIZE];
#endif

#endif
