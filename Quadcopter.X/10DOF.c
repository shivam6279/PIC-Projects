#include "10DOF.h"
#include "bitbang_I2C.h"
#include "settings.h"
#include <math.h>

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

XYZ VectorScale(XYZ a, float scale) {
    XYZ r;
    r.x = a.x * scale;
    r.y = a.y * scale;
    r.z = a.z * scale;
    return r;
}

//-----------------------------------------MPU6050---------------------------------
void MPU6050Init() {
    unsigned char i;
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x6B, 0x00}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x19, 0x07}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x1A, 0x03}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x1B, 0x00}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x1C, 0x00}, 2);
    for(i = 0x1D; i <= 0x23; i++) {
        I2C_WriteRegisters(0xD0, (unsigned char[2]){i, 0x00}, 2);
    }
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x24, 0x40}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x25, 0x8C}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x26, 0x02}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x27, 0x88}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x28, 0x0C}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x29, 0x0A}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x2A, 0x81}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x64, 0x01}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x67, 0x03}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x01, 0x80}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x34, 0x04}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x64, 0x00}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x6A, 0x00}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x64, 0x01}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x6A, 0x20}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x34, 0x13}, 2);
    for(i = 0; i < 5; i++) {
        acc_buffer[i].x = 0;
        acc_buffer[i].y = 0;
        acc_buffer[i].z = 0;
        gyro_buffer[i].x = 0;
        gyro_buffer[i].y = 0;
        gyro_buffer[i].z = 0;
    }
    gyro_avg.x = GYRO_X_OFFSET;
    gyro_avg.y = GYRO_Y_OFFSET;
    gyro_avg.z = GYRO_Z_OFFSET;
}

void GetAcc() {
    unsigned char temp[6];
    unsigned char i;
    if(IMU_BUFFER_SIZE > 0){
        for(i = (IMU_BUFFER_SIZE - 1); i >= 1; i--) {
            acc_buffer[i].x = acc_buffer[i - 1].x;
            acc_buffer[i].y = acc_buffer[i - 1].y;
            acc_buffer[i].z = acc_buffer[i - 1].z;
        }
    }
    I2C_ReadRegisters(0xD0, 0x3B, temp, 6);
    acc.y = (signed short)(temp[0] << 8 | temp[1]);
    acc.x = (signed short)(temp[2] << 8 | temp[3]) * (-1);
    acc.z = (signed short)(temp[4] << 8 | temp[5]) * (-1);
    if(IMU_BUFFER_SIZE > 0) {
        acc_buffer[0].x = acc.x;
        acc_buffer[0].y = acc.y;
        acc_buffer[0].z = acc.z;
        for(i = 1; i < IMU_BUFFER_SIZE; i++){
            acc.x += acc_buffer[i].x;
            acc.y += acc_buffer[i].y;
            acc.z += acc_buffer[i].z;
        }
        acc.x /= IMU_BUFFER_SIZE;
        acc.y /= IMU_BUFFER_SIZE;
        acc.z /= IMU_BUFFER_SIZE;
    }
}

void GetGyro() {
    unsigned char temp[6];
    unsigned char i;
    if(IMU_BUFFER_SIZE > 0){
        for(i = (IMU_BUFFER_SIZE - 1); i >= 1; i--) {
            gyro_buffer[i].x = gyro_buffer[i - 1].x;
            gyro_buffer[i].y = gyro_buffer[i - 1].y;
            gyro_buffer[i].z = gyro_buffer[i - 1].z;
        }
    }
    I2C_ReadRegisters(0xD0, 0x43, temp, 6);
    gyro.y = (signed short)(temp[0] << 8 | temp[1]);
    gyro.x = (signed short)(temp[2] << 8 | temp[3]);
    gyro.z = (signed short)(temp[4] << 8 | temp[5]);
    if(IMU_BUFFER_SIZE > 0){
        gyro_buffer[0].x = gyro.x;
        gyro_buffer[0].y = gyro.y;
        gyro_buffer[0].z = gyro.z;
        for(i = 1; i < IMU_BUFFER_SIZE; i++) {
            gyro.x += gyro_buffer[i].x;
            gyro.y += gyro_buffer[i].y;
            gyro.z += gyro_buffer[i].z;
        }
        gyro.x /= IMU_BUFFER_SIZE;
        gyro.y /= IMU_BUFFER_SIZE;
        gyro.z /= IMU_BUFFER_SIZE;
    }
    gyro.x = (gyro.x - gyro_avg.x) / GYRO_X_GAIN;
    gyro.y = (gyro.y - gyro_avg.y) / GYRO_Y_GAIN;
    gyro.z = (gyro.z - gyro_avg.z) / GYRO_Z_GAIN;
}

void CalibrateGyro() {
    unsigned char temp[6];
    int i;
    for(i = 0; i < 100; i++) {
        I2C_ReadRegisters(0xD0, 0x43, temp, 6);
        gyro_avg.y = (signed short)(temp[0] << 8 | temp[1]);
        gyro_avg.x = (signed short)(temp[2] << 8 | temp[3]);
        gyro_avg.z = (signed short)(temp[4] << 8 | temp[5]);
        delay_ms(1);
    }
    gyro_avg.x /= 100.0;
    gyro_avg.y /= 100.0;
    gyro_avg.z /= 100.0;
}

//-----------------------------------------HMC5883---------------------------------

void HMC5883Init() {
    unsigned char i;
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x6A, 0x00}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x37, 0x02}, 2);
    for(i = 0; i < 5; i++) {
        compass_buffer[i].x = 0;
        compass_buffer[i].y = 0;
        compass_buffer[i].z = 0;
    }
    I2C_WriteRegisters(0x3C, (unsigned char[2]){0, 0x14}, 2);
    I2C_WriteRegisters(0x3C, (unsigned char[2]){1, 0x20}, 2);
    I2C_WriteRegisters(0x3C, (unsigned char[2]){2, 0x00}, 2);
    
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x6A, 0x20}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x37, 0x00}, 2);
    for(i = 0; i < 5; i++) {
        compass_buffer[i].x = 0;
        compass_buffer[i].y = 0;
        compass_buffer[i].z = 0;
    }
}

void GetCompass() {
    unsigned char temp[6];
    unsigned char i;
    if(IMU_BUFFER_SIZE > 0){
        for(i = (IMU_BUFFER_SIZE - 1); i >= 1; i--) {
            compass_buffer[i].x = compass_buffer[i - 1].x;
            compass_buffer[i].y = compass_buffer[i - 1].y;
            compass_buffer[i].z = compass_buffer[i - 1].z;
        }
    }
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x6A, 0x00}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x37, 0x02}, 2);
    I2C_ReadRegisters(0x3C, 0x03, temp, 6);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x6A, 0x20}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x37, 0x00}, 2);
    compass.y = (signed short)(temp[0] << 8 | temp[1]);
    compass.z = (signed short)(temp[2] << 8 | temp[3]);
    compass.x = (signed short)(temp[4] << 8 | temp[5]);
    if(IMU_BUFFER_SIZE > 0) {
        compass_buffer[0].x = compass.x;
        compass_buffer[0].y = compass.y;
        compass_buffer[0].z = compass.z;
        for(i = 1; i < IMU_BUFFER_SIZE; i++) {
            compass.x += compass_buffer[i].x;
            compass.y += compass_buffer[i].y;
            compass.z += compass_buffer[i].z;
        }
        compass.x /= IMU_BUFFER_SIZE;
        compass.y /= IMU_BUFFER_SIZE;
        compass.z /= IMU_BUFFER_SIZE;
    }
    compass.x = (compass.x - COMPASS_X_OFFSET) * COMPASS_X_GAIN;
    compass.y = (compass.y - COMPASS_Y_OFFSET) * COMPASS_Y_GAIN;
    compass.z = (compass.z - COMPASS_Z_OFFSET) * COMPASS_Z_GAIN;
}
//----------------------------------------------------------------------
void GetRawIMU() {
    GetRawAcc();
    GetRawGyro();
    GetRawCompass();
}

void GetRawAcc() {
    unsigned char temp[6];
    I2C_ReadRegisters(0xD0, 0x3B, temp, 6);
    acc.y = (signed short)(temp[0] << 8 | temp[1]);
    acc.x = (signed short)(temp[2] << 8 | temp[3]) * (-1);
    acc.z = (signed short)(temp[4] << 8 | temp[5]) * (-1);
}

void GetRawGyro() {
    unsigned char temp[6];
    I2C_ReadRegisters(0xD0, 0x43, temp, 6);
    gyro.y = (signed short)(temp[0] << 8 | temp[1]);
    gyro.x = (signed short)(temp[2] << 8 | temp[3]);
    gyro.z = (signed short)(temp[4] << 8 | temp[5]);
}

void GetRawCompass() {
    unsigned char temp[6];
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x6A, 0x00}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x37, 0x02}, 2);
    I2C_ReadRegisters(0x3C, 0x03, temp, 6);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x6A, 0x20}, 2);
    I2C_WriteRegisters(0xD0, (unsigned char[2]){0x37, 0x00}, 2);
    compass.y = (signed short)(temp[0] << 8 | temp[1]);
    compass.z = (signed short)(temp[2] << 8 | temp[3]);
    compass.x = (signed short)(temp[4] << 8 | temp[5]);
}

//-----------------------------------------BMP180---------------------------------
#ifdef BMP180
void BMP180Init() {
    unsigned char temp[22];
    I2C_ReadRegisters(0xEE, 0xAA, temp, 22);
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
    I2C_WriteRegisters(0xEE, (unsigned char[2]){0xF4, 0x2E}, 2);
}

void StartPressureRead() {
    I2C_WriteRegisters(0xEE, (unsigned char[2]){0xF4, 0x34 + OVERSAMPLING * 64}, 2);
}

void ReadRawTemperature() {
    float X1, X2;
    unsigned char t[2];
    I2C_ReadRegisters(0xEE, 0xF6, t, 2);
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
    
    I2C_ReadRegisters(0xEE, 0xF6, t, 3);
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

    if(B7 < 0x80000000) p = (B7 * 2) / B4;
    else p = (B7 / B4) * 2;
    X1 = ((float)p / 256.0f) * ((float)p / 256.0f);
    X1 = (X1 * 3038) / 65536.0f;
    X2 = (-7357 * (float)p) / 65536.0f;

    p = p + ((X1 + X2 + 3791) / 16.0f);
    altitude = 44330 * (1.0 - pow((float)p / SEA_LEVEL_PRESSURE, 0.1903));
    return altitude;
}
#endif

#ifdef MS5611
void MS5611Init() {
    unsigned char temp[2];
    I2C_WriteRegisters(0xEE, (unsigned char[1]){0x1E}, 1);
    delay_ms(100);
    I2C_ReadRegisters(0xEE, 0xA2, temp, 2);
    MS5611_fc[0] = temp[0] << 8 | temp[1];
    I2C_ReadRegisters(0xEE, 0xA2 + 2, temp, 2);
    MS5611_fc[1] = temp[0] << 8 | temp[1];
    I2C_ReadRegisters(0xEE, 0xA2 + 4, temp, 2);
    MS5611_fc[2] = temp[0] << 8 | temp[1];
    I2C_ReadRegisters(0xEE, 0xA2 + 6, temp, 2);
    MS5611_fc[3] = temp[0] << 8 | temp[1];
    I2C_ReadRegisters(0xEE, 0xA2 + 8, temp, 2);
    MS5611_fc[4] = temp[0] << 8 | temp[1];
    I2C_ReadRegisters(0xEE, 0xA2 + 10, temp, 2);
    MS5611_fc[5] = temp[0] << 8 | temp[1];
    #if OVERSAMPLING == 0
        oversampling_delay = 1;
    #elif OVERSAMPLING == 1
        oversampling_delay = 2;
    #elif OVERSAMPLING == 2
        oversampling_delay = 3;
    #elif OVERSAMPLING == 3
        oversampling_delay = 5;
    #elif OVERSAMPLING == 4
        oversampling_delay = 10;
    #endif
}

void StartPressureRead() {
    I2C_WriteRegisters(0xEE, (unsigned char[1]){0x40 + OVERSAMPLING * 2}, 1);
}

unsigned long int ReadRawPressure() {
    unsigned char temp[3];
    unsigned long int r;
    I2C_ReadRegisters(0xEE, 0x00, temp, 3);
    r = (unsigned long int)((unsigned long int)temp[0] << 16 | (unsigned long int)temp[1] << 8 | (unsigned long int)temp[2]);
    return r;
}

void StartTemperatureRead() {
    I2C_WriteRegisters(0xEE, (unsigned char[1]){0x50 + OVERSAMPLING * 2}, 1);
}

unsigned long int ReadRawTemperature() {
    unsigned char temp[3];
    unsigned long int r;
    I2C_ReadRegisters(0xEE, 0x00, temp, 3);
    r = (unsigned long int)((unsigned long int)temp[0] << 16 | (unsigned long int)temp[1] << 8 | (unsigned long int)temp[2]);
    return r;
}

long int ComputePressure(unsigned long int d1, unsigned long int d2) {
    unsigned long int dt = d2 - (unsigned long int)MS5611_fc[4] * 256;
    long long int off = (long long int)MS5611_fc[1] * 65536 + (long long int)MS5611_fc[3] * dt / 128;
    long long int sens = (long long int)MS5611_fc[0] * 32768 + (long long int)MS5611_fc[2] * dt / 256;
    
    #if COMPENSATION == 1
        unsigned long int temp;
        temp = 2000 + (long long int)dt * MS5611_fc[5] / 8388608;
        long long int off2 = 0, sens2 = 0;
        if(temp < 2000) {
            off2 = 5 * ((temp - 2000) * (temp - 2000)) / 2;
            sens2 = 5 * ((temp - 2000) * (temp - 2000)) / 4;
        }
        if(temp < -1500) {
            off2 = off2 + 7 * ((temp + 1500) * (temp + 1500));
    	    sens2 = sens2 + 11 * ((temp + 1500) * (temp + 1500)) / 2;
        }
        off = off - off2;
        sens = sens - sens2;
    #endif
    
    unsigned long int p = (d1 * sens / 2097152 - off) / 32768;
    return p;
}

double ComputeTemperature(unsigned long int d2) {
    long int dt = d2 - (unsigned long int)MS5611_fc[4] * 256;
    long long int temp = 2000 + ((long long int)dt * MS5611_fc[5]) / 8388608;
    #if COMPENSATION == 1
        long int temp2;
        if(temp < 2000) {
            temp2 = (dt * dt) / (2 << 30);
        }
        temp = temp - temp2;
    #endif
    return (double)temp / 100.0;
}

long int GetPressure() {
    unsigned long int d1, d2;
    StartPressureRead();
    delay_ms(oversampling_delay);
    d1 = ReadRawPressure();
    StartTemperatureRead();
    delay_ms(oversampling_delay);
    d2 = ReadRawTemperature();    
    return ComputePressure(d1, d2);
}

double GetTemperature() {
    unsigned long int d2;
    StartTemperatureRead();
    delay_ms(oversampling_delay);
    d2 = ReadRawTemperature();
    return ComputeTemperature(d2);
}

double GetAltitude(unsigned long int pressure) {
    return (44330.0f * (1.0f - pow((double)pressure / 101325.0f, 0.1902949f)));
}
#endif
