#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <stdbool.h>

#define IMU_BUFFER_SIZE 0

#define ACC_GRAVITY 9.81f //m/s^2

#define ACC_X_OFFSET 0.0f
#define ACC_Y_OFFSET 0.0f
#define ACC_Z_OFFSET 0.0f

#define ACC_X_GAIN 1.0f
#define ACC_Y_GAIN 1.0f
#define ACC_Z_GAIN 1.0f

#define GYRO_X_OFFSET 740.0f
#define GYRO_Y_OFFSET -180.0f
#define GYRO_Z_OFFSET -50.0f

#define GYRO_X_GAIN 131.0f
#define GYRO_Y_GAIN 131.0f
#define GYRO_Z_GAIN 131.0f

typedef struct {
    float x;
    float y;
    float z;
} XYZ;

extern void dmpInit();
extern void Getdmp(float[4], XYZ*, float*, float*, float*);
extern void GetFIFO(unsigned char*);
extern void dmpDecode(unsigned char[42], float[4], XYZ*, float*, float*, float*);
extern unsigned int GetFIFOCount();

extern void VectorReset(XYZ *v);
extern XYZ VectorAdd(XYZ, XYZ);
extern XYZ VectorSubtract(XYZ, XYZ);
extern XYZ VectorScale(XYZ, float);

//MPU6050
#define MPU6050_ADDR 0x68

extern void MPU6050Init();
extern bool GetRawAcc(XYZ*);
extern bool GetAcc(XYZ*);
extern void CalibrateGyro();
extern bool GetRawGyro(XYZ*);
extern bool GetGyro(XYZ*);

extern XYZ acc_offset, acc_gain;
extern XYZ gyro_offset;

#if IMU_BUFFER_SIZE > 0
extern XYZ acc_buffer[IMU_BUFFER_SIZE], gyro_buffer[IMU_BUFFER_SIZE];
#endif

#endif
