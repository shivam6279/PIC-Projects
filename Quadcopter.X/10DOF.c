#include "10DOF.h"
#include "bitbang_I2C.h"
#include "settings.h"
#include <stdbool.h>
#include <math.h>
#include "pic32.h"

XYZ acc_offset, acc_gain;
XYZ gyro_offset, gyro_gain;
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
void MPU6050_Init() {
    unsigned char i;
    
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x6B, 0x00}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x19, 0x07}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x1A, 0x03}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x1B, 0x10}, 2);
    I2C_WriteRegisters(MPU6050_ADDR, (unsigned char[2]){0x1C, 0x00}, 2);
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

    acc_offset.x = ACC_X_OFFSET;
    acc_offset.y = ACC_Y_OFFSET;
    acc_offset.z = ACC_Z_OFFSET;

    acc_gain.x = ACC_X_GAIN;
    acc_gain.y = ACC_Y_GAIN;
    acc_gain.z = ACC_Z_GAIN;

    gyro_offset.x = GYRO_X_OFFSET;
    gyro_offset.y = GYRO_Y_OFFSET;
    gyro_offset.z = GYRO_Z_OFFSET;
    
    gyro_gain.x = GYRO_X_GAIN;
    gyro_gain.y = GYRO_Y_GAIN;
    gyro_gain.z = GYRO_Z_GAIN;
}

bool GetRawAcc(XYZ *acc) {
    unsigned char temp[6];
    if(!I2C_ReadRegisters(MPU6050_ADDR, 0x3B, temp, 6))
        return false;

    // Order: XH, XL, YH, YZ, ZH, ZL

    acc->y = -(signed short)(temp[0] << 8 | temp[1]); // x
    acc->x =  (signed short)(temp[2] << 8 | temp[3]); // y
    acc->z =  (signed short)(temp[4] << 8 | temp[5]); // z
    
    return true;
}

bool GetAcc(XYZ *acc) {
    XYZ acc_raw;
    
    if(!GetRawAcc(&acc_raw))
        return false;

    acc->x = (1.0 - ACC_LPF) * (acc_raw.x - acc_offset.x) * ACC_GRAVITY / 16384.0f * acc_gain.x + ACC_LPF * acc->x;
    acc->y = (1.0 - ACC_LPF) * (acc_raw.y - acc_offset.y) * ACC_GRAVITY / 16384.0f * acc_gain.y + ACC_LPF * acc->y;
    acc->z = (1.0 - ACC_LPF) * (acc_raw.z - acc_offset.z) * ACC_GRAVITY / 16384.0f * acc_gain.z + ACC_LPF * acc->z;

    return true;
}

bool GetRawGyro(XYZ *gyro) {
    unsigned char temp[6];
    if(!I2C_ReadRegisters(MPU6050_ADDR, 0x43, temp, 6))
        return false;

    // Order: XH, XL, YH, YZ, ZH, ZL

    gyro->y = -(signed short)(temp[0] << 8 | temp[1]);
    gyro->x =  (signed short)(temp[2] << 8 | temp[3]);
    gyro->z =  (signed short)(temp[4] << 8 | temp[5]);
    return true;
}

bool GetGyro(XYZ *gyro) { 
    XYZ gyro_raw;
    
    if(!GetRawGyro(&gyro_raw))
        return false;
    
    gyro->x = (1.0 - GYRO_LPF) * (gyro_raw.x - gyro_offset.x) / gyro_gain.x + GYRO_LPF * gyro->x;
    gyro->y = (1.0 - GYRO_LPF) * (gyro_raw.y - gyro_offset.y) / gyro_gain.y + GYRO_LPF * gyro->y;
    gyro->z = (1.0 - GYRO_LPF) * (gyro_raw.z - gyro_offset.z) / gyro_gain.z + GYRO_LPF * gyro->z;
    return true;
}

//-----------------------------------Magnetometer----------------------------------

void LIS3MDL_Init() {
    unsigned char i;
    
    I2C_WriteRegisters(LIS3MDL_ADDR, (unsigned char[2]){0x21, 0x60}, 2);    
    I2C_WriteRegisters(LIS3MDL_ADDR, (unsigned char[2]){0x20, 0x7E}, 2);    
    I2C_WriteRegisters(LIS3MDL_ADDR, (unsigned char[2]){0x23, 0x0C}, 2);
    I2C_WriteRegisters(LIS3MDL_ADDR, (unsigned char[2]){0x24, 0x00}, 2);
    I2C_WriteRegisters(LIS3MDL_ADDR, (unsigned char[2]){0x22, 0x00}, 2);
    
//    I2C_WriteRegisters(LIS3MDL_ADDR, (unsigned char[2]){0x30, 0x05}, 2);
    
    ComputeCompassOffsetGain((XYZ){COMPASS_X_MIN, COMPASS_Y_MIN, COMPASS_Z_MIN}, (XYZ){COMPASS_X_MAX, COMPASS_Y_MAX, COMPASS_Z_MAX});
}

bool GetRawCompass(XYZ *compass) {
    unsigned char temp[6];
    
    if(!I2C_ReadRegisters(LIS3MDL_ADDR, 0x28, temp, 6))
        return false;

    // Order: XH, XL, YH, YZ, ZH, ZL
    
    compass->y =  (signed short)(temp[1] << 8 | temp[0]); //x
    compass->x = -(signed short)(temp[3] << 8 | temp[2]); //y
    compass->z =  (signed short)(temp[5] << 8 | temp[4]); //z
    
    return true;
}

//----------------------------------------------------------------------

void ComputeCompassOffsetGain(XYZ c_min, XYZ c_max) {
    compass_offset.x = (c_max.x + c_min.x) / 2.0f;
    compass_offset.y = (c_max.y + c_min.y) / 2.0f;
    compass_offset.z = (c_max.z + c_min.z) / 2.0f;

    compass_gain.x = 2.0f / (c_max.x - c_min.x);
    compass_gain.y = 2.0f / (c_max.y - c_min.y);
    compass_gain.z = 2.0f / (c_max.z - c_min.z);
}

bool GetCompass(XYZ *compass) {
    XYZ comp_raw;
    if(!GetRawCompass(&comp_raw))
        return false;

    compass->x = (1.0 - COMPASS_LPF) * (comp_raw.x - compass_offset.x) * compass_gain.x + COMPASS_LPF * compass->x;
    compass->y = (1.0 - COMPASS_LPF) * (comp_raw.y - compass_offset.y) * compass_gain.y + COMPASS_LPF * compass->y;
    compass->z = (1.0 - COMPASS_LPF) * (comp_raw.z - compass_offset.z) * compass_gain.z + COMPASS_LPF * compass->z;
    
    return true;
}

//------------------------------------Altimeter-----------------------------------

unsigned char oversampling_delay;

//-------------------------------------BMP390-------------------------------------

static signed short ac1, ac2, ac3, b1, b2, mb, mc, md;
static unsigned short ac4, ac5, ac6;
static unsigned long int UT;
static float B5;

void BMP390_Init() {
    unsigned char temp[22];
    I2C_ReadRegisters(BMP390_ADDR, 0xAA, temp, 22);
    ac1 = (signed short)(temp[0] << 8 | temp[1]);
    ac2 = (signed short)(temp[2] << 8 | temp[3]);
    ac3 = (signed short)(temp[4] << 8 | temp[5]);
    ac4 = temp[6] << 8 | temp[7];
    ac5 = temp[8] << 8 | temp[9];
    ac6 = temp[10] << 8 | temp[11];
    b1 = (signed short)(temp[12] << 8 | temp[13]);
    b2 = (signed short)(temp[14] << 8 | temp[15]);
    mb = (signed short)(temp[16] << 8 | temp[17]);
    mc = (signed short)(temp[18] << 8 | temp[19]);
    md = (signed short)(temp[20] << 8 | temp[21]);
    #if OVERSAMPLING == 0
        oversampling_delay = 5;
    #elif OVERSAMPLING == 1
        oversampling_delay = 8;
    #elif OVERSAMPLING == 2
        oversampling_delay = 14;
    #elif OVERSAMPLING == 3
        oversampling_delay = 26;
    #endif
}

void StartTemperatureRead() {
    I2C_WriteRegisters(BMP390_ADDR, (unsigned char[2]){0xF4, 0x2E}, 2);
}

void StartPressureRead() {
    I2C_WriteRegisters(BMP390_ADDR, (unsigned char[2]){0xF4, 0x34 + OVERSAMPLING * 64}, 2);
}

void ReadRawTemperature() {
    float X1, X2;
    unsigned char t[2];
    I2C_ReadRegisters(BMP390_ADDR, 0xF6, t, 2);
    UT = t[0] << 8 | t[1];
    X1 = ((float)UT - (float)ac6) * ((float)ac5 / 32768.0f);
    X2 = ((float)(mc * 2048.0f)) / (X1 + (float)md);
    B5 = X1 + X2;
}

double ComputeTemperature() {
    float temp;
    temp = (B5 + 8) / 16.0f;
    temp /= 10;
    return temp;
}

float GetAltitude() {
    long int UP, p;
    float altitude, X1, X2, X3, B3, B6;
    unsigned long int B4, B7;
    unsigned char t[3];
    
    I2C_ReadRegisters(BMP390_ADDR, 0xF6, t, 3);
    UP = ((t[0] << 16) | (t[1] << 8) | (t[2]));
    UP >>= (8 - OVERSAMPLING);

    // do pressure calcs
    B6 = B5 - 4000;
    X1 = ((float)b2 * ((B6 * B6) / 4096.0f)) / 2048.0f;
    X2 = ((float)ac2 * B6) / 2048.0f;
    X3 = X1 + X2;
    B3 = (((float)ac1 * 4 + X3) * pow(2, OVERSAMPLING) + 2) / 4;

    X1 = ((float)ac3 * B6) / 8192.0f;
    X2 = ((float)b1 * ((B6 * B6) / 4096.0f)) / 65536.0f;
    X3 = ((X1 + X2) + 2)/ 4.0f;
    B4 = (float)ac4 * ((X3 + 32768)) / 32768.0f;
    B7 = ((float)UP - B3) * (50000 / pow(2, OVERSAMPLING));

    if(B7 < 0x80000000) 
        p = (B7 * 2) / B4;
    else 
        p = (B7 / B4) * 2;
    X1 = ((float)p / 256.0f) * ((float)p / 256.0f);
    X1 = (X1 * 3038) / 65536.0f;
    X2 = (-7357 * (float)p) / 65536.0f;

    p = p + ((X1 + X2 + 3791) / 16.0f);
    altitude = 44330 * (1.0 - pow((float)p / SEA_LEVEL_PRESSURE, 0.1903));
    return altitude;
}
