#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <stdbool.h>

#define IMU_BUFFER_SIZE 10

#define ACC_GRAVITY 9.81f //m/s^2

#define ACC_X_OFFSET 0.0f
#define ACC_Y_OFFSET 0.0f
#define ACC_Z_OFFSET 0.0f

#define GYRO_X_OFFSET -62.0f
#define GYRO_Y_OFFSET -1.0f
#define GYRO_Z_OFFSET 82.0f

#define COMPASS_X_MIN -4255.0f
#define COMPASS_X_MAX 5930.0f

#define COMPASS_Y_MIN -5655.0f
#define COMPASS_Y_MAX 4126.0f

#define COMPASS_Z_MIN -5655.0f
#define COMPASS_Z_MAX 5406.0f

#define MPU9250_ACC_GYRO_ADDR 0x68
#define MPU9250_MAG_ADDR 0x0C

typedef struct {
    float x;
    float y;
    float z;
} XYZ;

extern void VectorReset(XYZ *v);
extern XYZ VectorAdd(XYZ, XYZ);
extern XYZ VectorSubtract(XYZ, XYZ);
extern XYZ VectorScale(XYZ, float);

extern bool MPU9250ReadRaw(XYZ*, XYZ*, XYZ*);
extern bool GetRawAcc(XYZ*);
extern bool GetAcc(XYZ*);
extern bool GetRawGyro(XYZ*);
extern bool GetGyro(XYZ*);

extern void ComputeCompassOffsetGain(XYZ, XYZ);
extern bool GetRawCompass(XYZ*);
extern bool GetCompass(XYZ*);

extern void writeMPU9250Register(unsigned char, unsigned char);
extern void writeAK8963Register(unsigned char, unsigned char);
extern void readAK8963Registers(unsigned char, unsigned char*, unsigned char);

//Pressure sensors

extern unsigned char oversampling_delay;

extern XYZ acc_offset, acc_gain;
extern XYZ gyro_offset, gyro_gain;
extern XYZ compass_offset, compass_gain;

#if IMU_BUFFER_SIZE > 0
extern XYZ acc_buffer[IMU_BUFFER_SIZE], gyro_buffer[IMU_BUFFER_SIZE], compass_buffer[IMU_BUFFER_SIZE];
#endif

#endif
