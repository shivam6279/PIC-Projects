#include "10DOF.h"
#include "bitbang_I2C.h"
#include "settings.h"
#include <stdbool.h>
#include <math.h>
#include "pic32.h"

//I2C sensors
#include "MPU6050.h"
#include "LIS3MDL.h"
#include "QMC5883.h"
#include "BMP390.h"

XYZ acc_offset, acc_gain;
XYZ gyro_offset, gyro_gain;
XYZ compass_offset, compass_gain;

float acc_axes[3][3] = {    { 0,  1,  0},
                            {-1,  0,  0},
                            { 0,  0,  1}    };

float gyro_axes[3][3] = {   { 0,  1,  0},
                            {-1,  0,  0},
                            { 0,  0,  1}    };

float comp_axes[3][3] = {   { 1,  0,  0},
                            { 0, -1,  0},
                            { 0,  0, -1}    };

bool Init10DOF() {
    MPU6050_Init();
#ifdef QMC5883
    QMC5883_Init();
#endif
#ifdef LIS3MDL
    LIS3MDL_Init();
#endif
    
    BMP390_Init();
    
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
    
    ComputeCompassOffsetGain((XYZ){COMPASS_X_MIN, COMPASS_Y_MIN, COMPASS_Z_MIN}, (XYZ){COMPASS_X_MAX, COMPASS_Y_MAX, COMPASS_Z_MAX});
}

//Accelerometer
bool GetRawAcc(XYZ *acc) { 
#ifdef MPU6050
    if(!MPU6050_GetAcc(&acc->x, &acc->y, &acc->z))
        return false;
    *acc = VectorTransform(*acc, acc_axes);
#else
    return false
#endif

    return true;
}

bool GetAcc(XYZ *acc) {
    XYZ acc_raw;
    
    GetRawAcc(&acc_raw);

    acc->x = (acc_raw.x - acc_offset.x) * ACC_GRAVITY * acc_gain.x;
    acc->y = (acc_raw.y - acc_offset.y) * ACC_GRAVITY * acc_gain.y;
    acc->z = (acc_raw.z - acc_offset.z) * ACC_GRAVITY * acc_gain.z;

    return true;
}

//Gyroscope
bool GetRawGyro(XYZ *gyro) { 
#ifdef MPU6050
    if(!MPU6050_GetGyro(&gyro->x, &gyro->y, &gyro->z))
        return false;
    *gyro = VectorTransform(*gyro, gyro_axes);
#else
    return false
#endif
    return true;
}

bool GetGyro(XYZ *gyro) { 
    XYZ gyro_raw;
    
    GetRawGyro(&gyro_raw);
    
    gyro->x = (gyro_raw.x - gyro_offset.x) * gyro_gain.x;
    gyro->y = (gyro_raw.y - gyro_offset.y) * gyro_gain.y;
    gyro->z = (gyro_raw.z - gyro_offset.z) * gyro_gain.z;
    return true;
}

//Magnetometer

float compass_B[3] = {-1810.85, 2362.38, -812.67};

 float compass_Ainv[3][3] = {   {  1.14376, -0.00286, -0.01164},
                                { -0.00286,  1.15467, -0.01392},
                                { -0.01164, -0.01392,  1.16204} };

void ComputeCompassOffsetGain(XYZ c_min, XYZ c_max) {
    compass_offset.x = (c_max.x + c_min.x) / 2.0f;
    compass_offset.y = (c_max.y + c_min.y) / 2.0f;
    compass_offset.z = (c_max.z + c_min.z) / 2.0f;

    compass_gain.x = 2.0f / (c_max.x - c_min.x);
    compass_gain.y = 2.0f / (c_max.y - c_min.y);
    compass_gain.z = 2.0f / (c_max.z - c_min.z);
}

bool GetRawCompass(XYZ *comp) {
#if defined(LIS3MDL)
    if(!LIS3MDL_GetCompass(&comp->x, &comp->y, &comp->z))
        return false;
    *comp = VectorTransform(*comp, comp_axes);
#elif defined(QMC5883)
    if(!QMC5883_GetCompass(&comp->x, &comp->y, &comp->z))
        return false;
    *comp = VectorTransform(*comp, comp_axes);
#else
    return false;
#endif
    
    return true;
}

bool GetCompass(XYZ *compass) {
    XYZ comp_raw;

    GetRawCompass(&comp_raw);

//    compass->x = (comp_raw.x - compass_offset.x) * compass_gain.x;
//    compass->y = (comp_raw.y - compass_offset.y) * compass_gain.y;
//    compass->z = (comp_raw.z - compass_offset.z) * compass_gain.z;
    
    compass->x = comp_raw.x - compass_B[0];
    compass->y = comp_raw.y - compass_B[1];
    compass->z = comp_raw.z - compass_B[2];
    
    compass->x = compass_Ainv[0][0] * compass->x + compass_Ainv[0][1] * compass->y + compass_Ainv[0][2] * compass->z;
    compass->y = compass_Ainv[1][0] * compass->x + compass_Ainv[1][1] * compass->y + compass_Ainv[1][2] * compass->z;
    compass->z = compass_Ainv[2][0] * compass->x + compass_Ainv[2][1] * compass->y + compass_Ainv[2][2] * compass->z;
    
    return true;
}

//Altimeter
float PressureToAltitude(float pressure) {
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

float GetAltitude() {
#ifdef BMP390
    unsigned long temperature, pressure;
    BMP390_read_pressure_temp(&pressure, &temperature);
    temperature = BMP390_compensate_temp(temperature);
    pressure = BMP390_compensate_pressure(pressure, temperature);
    return PressureToAltitude(pressure);
#else
    return 0;
#endif
}

// Vector math
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
 
XYZ VectorTransform(XYZ a, float t[3][3]) {
    XYZ r;
    r.x = t[0][0] * a.x + t[0][1] * a.y + t[0][2] * a.z;
    r.y = t[1][0] * a.x + t[1][1] * a.y + t[1][2] * a.z;
    r.z = t[2][0] * a.x + t[2][1] * a.y + t[2][2] * a.z;
    return r;
}