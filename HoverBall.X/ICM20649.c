#include "ICM20649.h"
#include "bitbang_I2C.h"
#include <stdbool.h>
#include <math.h>
#include "pic32.h"

XYZ acc_offset, acc_gain;
XYZ gyro_offset;
XYZ compass_offset, compass_gain;

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
void ICM20649Init() {
    unsigned char i;

    BankSel(0);

    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x06, 0x80}, 2);

    delay_ms(100);

    BankSel(0);

    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x03, 0x00}, 2);
    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x06, 0x01}, 2);

    BankSel(0);

    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x0F, 0xC0}, 2);
    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x07, 0x00}, 2);

    BankSel(2);

    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x01, 0x3F}, 2);
    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x00, 5}, 2);
    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x02, 0x00}, 2);

    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x14, 0x39}, 2);
    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x10, 0}, 2);
    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x11, 5}, 2);
    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x15, 0x00}, 2);

    BankSel(0);

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
    if(!I2C_ReadRegisters(ICM20649_ADDR, 0x2D, temp, 6))
        return false;

    // Order: XH, XL, YH, YZ, ZH, ZL

    acc->x = (signed short)(temp[0] << 8 | temp[1]);
    acc->y = (signed short)(temp[2] << 8 | temp[3]);
    acc->z = (signed short)(temp[4] << 8 | temp[5]);
    
    return true;
}

bool GetAcc(XYZ *acc) {
    if(!GetRawAcc(acc))
        return false;

    acc->x = (acc->x - acc_offset.x) * ACC_GRAVITY / 16384.0f * acc_gain.x;
    acc->y = (acc->y - acc_offset.y) * ACC_GRAVITY / 16384.0f * acc_gain.y;
    acc->z = (acc->z - acc_offset.z) * ACC_GRAVITY / 16384.0f * acc_gain.z;

    return true;
}

bool GetRawGyro(XYZ *gyro) {
    unsigned char temp[6];

    BankSel(0);    
    if(!I2C_ReadRegisters(ICM20649_ADDR, 0x33, temp, 6))
        return false;

    // Order: XH, XL, YH, YZ, ZH, ZL

    gyro->x = (signed short)(temp[0] << 8 | temp[1]);
    gyro->y = (signed short)(temp[2] << 8 | temp[3]);
    gyro->z = (signed short)(temp[4] << 8 | temp[5]);
    return true;
}

bool GetGyro(XYZ *gyro) { 
    if(!GetRawGyro(gyro))
        return false;

    gyro->x = (gyro->x - gyro_offset.x) / GYRO_X_GAIN;
    gyro->y = (gyro->y - gyro_offset.y) / GYRO_Y_GAIN;
    gyro->z = (gyro->z - gyro_offset.z) / GYRO_Z_GAIN;
    return true;
}

void GetGyroOffsets() {
    XYZ gyro_avg = {0, 0, 0}, gyro;
    unsigned int i;
    
    for(i = 0 ; i < 500; i++) {
        GetRawGyro(&gyro);
        gyro_avg.x += gyro.x;
        gyro_avg.y += gyro.y;
        gyro_avg.z += gyro.z;
        delay_ms(2);
    }
    gyro_offset.x = gyro_avg.x / 500.0;
    gyro_offset.y = gyro_avg.y / 500.0;
}

void BankSel(unsigned char b) {
    I2C_WriteRegisters(ICM20649_ADDR, (unsigned char[2]){0x7F, b << 4}, 2);
}