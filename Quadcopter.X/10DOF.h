#ifndef _10DOF_H_
#define _10DOF_H_

#include "settings.h"
#include <stdbool.h>

#define IMU_BUFFER_SIZE 0

#define GYRO_X_OFFSET -260
#define GYRO_Y_OFFSET -470
#define GYRO_Z_OFFSET 45

#define GYRO_X_GAIN 98
#define GYRO_Y_GAIN -98
#define GYRO_Z_GAIN 106

#define COMPASS_X_MIN -4127.0f
#define COMPASS_X_MAX 5292.0f

#define COMPASS_Y_MIN -6046.0f
#define COMPASS_Y_MAX 2360.0f

#define COMPASS_Z_MIN -5420.0f
#define COMPASS_Z_MAX 3117.0f

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
extern void MPU6050Init();
extern XYZ GetRawAcc();
extern XYZ GetAcc();
extern XYZ GetRawGyro();
extern XYZ GetGyro();

//Magnetometer

//HMC5883
#ifdef HMC5883
void HMC5883Init();
#endif

//QMC5883
#ifdef QMC5883
void QMC5883Init();
bool QMC5883DataRdy();
#endif

extern void ComputeCompassOffsetGain(XYZ, XYZ);
extern XYZ GetRawCompass();
extern XYZ GetCompass();

//Pressure sensors

extern unsigned char oversampling_delay;

//BMP180
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

extern XYZ gyro_offset, compass_offset, compass_gain;

#if IMU_BUFFER_SIZE > 0
extern XYZ acc_buffer[IMU_BUFFER_SIZE], gyro_buffer[IMU_BUFFER_SIZE], compass_buffer[IMU_BUFFER_SIZE];
#endif

#endif
