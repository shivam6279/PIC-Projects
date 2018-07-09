#ifndef _10DOF_H_
#define _10DOF_H_

#include "settings.h"

#define GYRO_X_OFFSET 100
#define GYRO_Y_OFFSET -230
#define GYRO_Z_OFFSET -70

#define GYRO_X_GAIN 98
#define GYRO_Y_GAIN -98
#define GYRO_Z_GAIN 106

#define COMPASS_X_MIN -120.0f
#define COMPASS_X_MAX 300.0f

#define COMPASS_Y_MIN -70.0f
#define COMPASS_Y_MAX 280.0f

#define COMPASS_Z_MIN -750.0f
#define COMPASS_Z_MAX 25.0f

#define COMPASS_X_OFFSET (COMPASS_X_MAX + COMPASS_X_MIN) / 2.0f
#define COMPASS_Y_OFFSET (COMPASS_Y_MAX + COMPASS_Y_MIN) / 2.0f
#define COMPASS_Z_OFFSET (COMPASS_Z_MAX + COMPASS_Z_MIN) / 2.0f
#define COMPASS_X_GAIN 1.0f / ((COMPASS_X_MAX - COMPASS_X_MIN) / 2.0f)
#define COMPASS_Y_GAIN -1.0f / ((COMPASS_Y_MAX - COMPASS_Y_MIN) / 2.0f)
#define COMPASS_Z_GAIN 1.0f / ((COMPASS_Z_MAX - COMPASS_Z_MIN) / 2.0f)

#define BUFFER_SIZE 10

#ifdef BMP180
#define OVERSAMPLING 3  //0 - 3
#define SEA_LEVEL_PRESSURE 101325UL
#endif
#ifdef MS5611
#define OVERSAMPLING 4  //0 - 4
#define COMPENSATION 0
#endif

typedef struct{
    float x,y,z;
} XYZ;

extern void VectorReset(XYZ *v);

//MPU6050
extern void MPU6050Init();
extern void GetAcc();
extern void GetGyro();
extern void CalibrateGyro();
//HMC5883
extern void HMC5883Init();
extern void GetCompass();

extern void GetRawIMU();

//Pressure sensors
unsigned char oversampling_delay;
//BMP180
#ifdef BMP180
extern void BMP180Init();
extern void StartPressureRead();
extern void ReadRawPressure();
extern void StartTemperatureRead();
extern void ReadRawTemperature();
extern double ComputeTemperature();
extern float GetAltitude();

signed short ac1, ac2, ac3, b1, b2, mb, mc, md;
unsigned short ac4, ac5, ac6;
unsigned long int UT;
float B5;
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

unsigned int MS5611_fc[6];
#endif

XYZ acc, acc_buffer[BUFFER_SIZE];
XYZ gyro, gyro_buffer[BUFFER_SIZE], gyro_avg;
XYZ compass, compass_buffer[BUFFER_SIZE];

#endif
