#ifndef _10DOF_H_
#define _10DOF_H_

#include "settings.h"

#ifdef micro

#define GYRO_X_OFFSET 100
#define GYRO_Y_OFFSET -245
#define GYRO_Z_OFFSET -15

#define GYRO_X_GAIN 98
#define GYRO_Y_GAIN -98
#define GYRO_Z_GAIN 106

#define COMPASS_X_MIN -920.0f
#define COMPASS_X_MAX -230.0f

#define COMPASS_Y_MIN -130.0f
#define COMPASS_Y_MAX 550.0f

#define COMPASS_Z_MIN -600.0f
#define COMPASS_Z_MAX 0.0f

#define COMPASS_X_OFFSET (COMPASS_X_MAX + COMPASS_X_MIN) / 2.0f
#define COMPASS_Y_OFFSET (COMPASS_Y_MAX + COMPASS_Y_MIN) / 2.0f 
#define COMPASS_Z_OFFSET (COMPASS_Z_MAX + COMPASS_Z_MIN) / 2.0f 
#define COMPASS_X_GAIN (500.0f / ((COMPASS_X_MAX - COMPASS_X_MIN) / 2.0f))
#define COMPASS_Y_GAIN -(500.0f / ((COMPASS_Y_MAX - COMPASS_Y_MIN) / 2.0f))
#define COMPASS_Z_GAIN (500.0f / ((COMPASS_Z_MAX - COMPASS_Z_MIN) / 2.0f))
#endif

#ifdef mini

#define GYRO_X_OFFSET -32
#define GYRO_Y_OFFSET -310
#define GYRO_Z_OFFSET -45

#define GYRO_X_GAIN 98
#define GYRO_Y_GAIN -98
#define GYRO_Z_GAIN 106

#define COMPASS_X_MIN -340.0f
#define COMPASS_X_MAX 600.0f

#define COMPASS_Y_MIN -320.0f
#define COMPASS_Y_MAX 439.0f

#define COMPASS_Z_MIN -360.0f
#define COMPASS_Z_MAX 180.0f

#define COMPASS_X_OFFSET (COMPASS_X_MAX + COMPASS_X_MIN) / 2
#define COMPASS_Y_OFFSET (COMPASS_Y_MAX + COMPASS_Y_MIN) / 2
#define COMPASS_Z_OFFSET (COMPASS_Z_MAX + COMPASS_Z_MIN) / 2
#define COMPASS_X_GAIN (500.0f / (COMPASS_X_MAX - COMPASS_X_MIN) / 2)
#define COMPASS_Y_GAIN -(500.0f / (COMPASS_Y_MAX - COMPASS_Y_MIN) / 2)
#define COMPASS_Z_GAIN (500.0f / (COMPASS_Z_MAX - COMPASS_Z_MIN) / 2)
#endif

//#define big 1 

#ifdef big

#define GYRO_X_OFFSET 110
#define GYRO_Y_OFFSET -245
#define GYRO_Z_OFFSET -78

#define GYRO_X_GAIN 98
#define GYRO_Y_GAIN -98
#define GYRO_Z_GAIN 106

#define COMPASS_X_MIN 110.0f
#define COMPASS_X_MAX 470.0f

#define COMPASS_Y_MIN -230.0f
#define COMPASS_Y_MAX 90.0f

#define COMPASS_Z_MIN -840.0f
#define COMPASS_Z_MAX 50.0f

#define COMPASS_X_OFFSET (COMPASS_X_MAX + COMPASS_X_MIN) / 2
#define COMPASS_Y_OFFSET (COMPASS_Y_MAX + COMPASS_Y_MIN) / 2
#define COMPASS_Z_OFFSET (COMPASS_Z_MAX + COMPASS_Z_MIN) / 2
#define COMPASS_X_GAIN (500.0f / (COMPASS_X_MAX - COMPASS_X_MIN) / 2)
#define COMPASS_Y_GAIN -(500.0f / (COMPASS_Y_MAX - COMPASS_Y_MIN) / 2)
#define COMPASS_Z_GAIN (500.0f / (COMPASS_Z_MAX - COMPASS_Z_MIN) / 2)
#endif

#define BUFFER_SIZE 10
#define OVERSAMPLING 3
#define SEA_LEVEL_PRESSURE 101325UL

typedef struct{
    float x,y,z;
} XYZ;

//MPU6050
extern void MPU6050Init();
extern void GetAcc();
extern void GetGyro();
//HMC5883
extern void HMC5883Init();
extern void GetCompass();
//BMP180
extern void BMP180Init();
extern void GetRawTemperature();
extern float GetAltitude();

extern void GetRawIMU();

XYZ acc, acc_buffer[BUFFER_SIZE];
XYZ gyro, gyro_buffer[BUFFER_SIZE];
XYZ compass, compass_buffer[BUFFER_SIZE];

signed short ac1, ac2, ac3, b1, b2, mb, mc, md;
unsigned short ac4, ac5, ac6;
unsigned long int UT;
float B5;

#endif
