#ifndef _ICM20649_H_
#define _ICM20649_H_

#include <stdbool.h>

#define ACC_GRAVITY 9.81f //m/s^2

#define ACC_X_OFFSET 0.0f
#define ACC_Y_OFFSET 0.0f
#define ACC_Z_OFFSET 0.0f

#define ACC_X_GAIN 1.0f
#define ACC_Y_GAIN 1.0f
#define ACC_Z_GAIN 1.0f

#define GYRO_X_OFFSET 0.0f
#define GYRO_Y_OFFSET 0.0f
#define GYRO_Z_OFFSET 0.0f

#define GYRO_X_GAIN 8.192f
#define GYRO_Y_GAIN 8.192f
#define GYRO_Z_GAIN 8.192f

typedef struct {
    float x;
    float y;
    float z;
} XYZ;

extern void VectorReset(XYZ *v);
extern XYZ VectorAdd(XYZ, XYZ);
extern XYZ VectorSubtract(XYZ, XYZ);
extern XYZ VectorScale(XYZ, float);

#define ICM20649_ADDR 0x68

extern void ICM20649Init();
extern bool GetRawAcc(XYZ*);
extern bool GetAcc(XYZ*);
extern bool GetRawGyro(XYZ*);
extern bool GetGyro(XYZ*);
extern void GetGyroOffsets();

extern void BankSel(unsigned char);

extern XYZ acc_offset, acc_gain;
extern XYZ gyro_offset;

#endif
