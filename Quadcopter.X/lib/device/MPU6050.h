#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "settings.h"
#include <stdbool.h>

#define MPU6050_ADDR 0x68

extern void MPU6050_Init();
extern bool MPU6050_GetAcc(float*, float*, float*);
extern bool MPU6050_GetGyro(float*, float*, float*);

#endif
