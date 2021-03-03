#include "MPU6050.h"
#include "bitbang_I2C.h"
#include "EEPROM.h"
#include <stdbool.h>
#include <math.h>

XYZ acc_offset, acc_gain;
XYZ gyro_offset;
XYZ compass_offset, compass_gain;

#if IMU_BUFFER_SIZE > 0
XYZ acc_buffer[IMU_BUFFER_SIZE], gyro_buffer[IMU_BUFFER_SIZE], compass_buffer[IMU_BUFFER_SIZE];
#endif

void VectorReset(XYZ *v) {
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
}

XYZ VectorAdd(XYZ a, XYZ b) {
    XYZ r;
    r.x = a.x + b.x;
    r.y = a.y + b.y;
    r.z = a.z + b.z;
    return r;
}

XYZ VectorSubtract(XYZ a, XYZ b) {
    XYZ r;
    r.x = a.x - b.x;
    r.y = a.y - b.y;
    r.z = a.z - b.z;
    return r;
}

XYZ VectorScale(XYZ a, float scale) {
    XYZ r;
    r.x = a.x * scale;
    r.y = a.y * scale;
    r.z = a.z * scale;
    return r;
}

//---------------------------------------MPU6050-----------------------------------
void MPU6050Init() {
    unsigned char i;
    
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x6B, 0x00}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x19, 0x07}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x1A, 0x03}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x1B, 0x00}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x1C, 0x00}, 2);
    for(i = 0x1D; i <= 0x23; i++) 
        I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){i, 0x00}, 2);
    
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
    
#if IMU_BUFFER_SIZE > 0
    for(i = 0; i < IMU_BUFFER_SIZE; i++) {
        acc_buffer[i] = (XYZ){0.0, 0.0, 0.0};
        gyro_buffer[i] = (XYZ){0.0, 0.0, 0.0};
    }
#endif

    acc_offset.x = ACC_X_OFFSET;
    acc_offset.y = ACC_Y_OFFSET;
    acc_offset.z = ACC_Z_OFFSET;

    acc_gain.x = ACC_X_GAIN;
    acc_gain.y = ACC_Y_GAIN;
    acc_gain.z = ACC_Z_GAIN;

    gyro_offset.x = GYRO_X_OFFSET;
    gyro_offset.y = GYRO_Y_OFFSET;
    gyro_offset.z = GYRO_Z_OFFSET;
}

bool GetRawAcc(XYZ *acc) {
    unsigned char temp[6];
    if(!I2C_ReadRegisters(MPU6050_ADDR, 0x3B, temp, 6))
        return false;

    // Order: XH, XL, YH, YZ, ZH, ZL

    acc->x = -(signed short)(temp[0] << 8 | temp[1]);
    acc->y =  (signed short)(temp[2] << 8 | temp[3]);
    acc->z = -(signed short)(temp[4] << 8 | temp[5]);
    
    return true;
}

bool GetAcc(XYZ *acc) {
    if(!GetRawAcc(acc))
        return false;
    
#if IMU_BUFFER_SIZE > 0
    unsigned char i;    
    for(i = (IMU_BUFFER_SIZE - 1); i >= 1; i--)
        acc_buffer[i] = acc_buffer[i - 1];

    acc_buffer[0] = *acc;
    for(i = 1; i < IMU_BUFFER_SIZE; i++)
        *acc = VectorAdd(*acc, acc_buffer[i]);
    
    *acc = VectorScale(*acc, 1.0f / (float)IMU_BUFFER_SIZE);
#endif

    acc->x = (acc->x - acc_offset.x) * ACC_GRAVITY / 16384.0f * acc_gain.x;
    acc->y = (acc->y - acc_offset.y) * ACC_GRAVITY / 16384.0f * acc_gain.y;
    acc->z = (acc->z - acc_offset.z) * ACC_GRAVITY / 16384.0f * acc_gain.z;

    return true;
}

bool GetRawGyro(XYZ *gyro) {
    unsigned char temp[6];
    if(!I2C_ReadRegisters(MPU6050_ADDR, 0x43, temp, 6))
        return false;

    // Order: XH, XL, YH, YZ, ZH, ZL

    gyro->x = -(signed short)(temp[0] << 8 | temp[1]);
    gyro->y =  (signed short)(temp[2] << 8 | temp[3]);
    gyro->z = -(signed short)(temp[4] << 8 | temp[5]);
    return true;
}

bool GetGyro(XYZ *gyro) { 
    if(!GetRawGyro(gyro))
        return false;
    
#if IMU_BUFFER_SIZE > 0
    unsigned char i;
    
    for(i = (IMU_BUFFER_SIZE - 1); i >= 1; i--)
        gyro_buffer[i] = gyro_buffer[i - 1];

    gyro_buffer[0] = *gyro;
    for(i = 1; i < IMU_BUFFER_SIZE; i++)
        *gyro = VectorAdd(*gyro, gyro_buffer[i]);

    *gyro = VectorScale(*gyro, 1.0f / (float)IMU_BUFFER_SIZE);
#endif
    gyro->x = (gyro->x - gyro_offset.x) / GYRO_X_GAIN;
    gyro->y = (gyro->y - gyro_offset.y) / GYRO_Y_GAIN;
    gyro->z = (gyro->z - gyro_offset.z) / GYRO_Z_GAIN;
    return true;
}