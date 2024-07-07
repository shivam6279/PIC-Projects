#include "MPU6050.h"
#include "bitbang_I2C.h"
#include <stdbool.h>
#include <math.h>
#include "pic32.h"

void MPU6050_Init() {
    unsigned char i;
    
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x6B, 0x00}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x19, 0x07}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x1A, 0x00}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x1B, 0x18}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x1C, 0x10}, 2);
    for(i = 0x1D; i <= 0x23; i++) 
        I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){i, 0x00}, 2);
    
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x37, 0x00}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x38, 0x11}, 2);
    
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x24, 0x40}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x25, 0x8C}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x26, 0x02}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x27, 0x88}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x28, 0x0C}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x29, 0x0A}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x2A, 0x81}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x64, 0x01}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x67, 0x03}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x01, 0x80}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x34, 0x04}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x64, 0x00}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x6A, 0x00}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x64, 0x01}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x6A, 0x20}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x34, 0x13}, 2);
}

bool MPU6050_GetAcc(float *x, float *y, float *z) {
    unsigned char temp[6];
    if(!I2C_ReadRegisters(MPU6050_ADDR, 0x3B, temp, 6))
        return false;

    // Order: XH, XL, YH, YZ, ZH, ZL

    *x = (signed short)(temp[0] << 8 | temp[1]);
    *y = (signed short)(temp[2] << 8 | temp[3]);
    *z = (signed short)(temp[4] << 8 | temp[5]);

    return true;
}

bool MPU6050_GetGyro(float *x, float *y, float *z) {
    unsigned char temp[6];
    if(!I2C_ReadRegisters(MPU6050_ADDR, 0x43, temp, 6))
        return false;

    // Order: XH, XL, YH, YZ, ZH, ZL

    *x = (signed short)(temp[0] << 8 | temp[1]);
    *y = (signed short)(temp[2] << 8 | temp[3]);
    *z = (signed short)(temp[4] << 8 | temp[5]);
    return true;
}