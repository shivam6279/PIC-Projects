#include "10DOF.h"
#include "bitbang_i2c.h"
#include <math.h>

//-----------------------------------------MPU6050---------------------------------
void MPU6050Init(){
    unsigned char i;
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6B, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x19, 0x07}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x1A, 0x03}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x1B, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x1C, 0x00}, 2);
    for(i = 0x1D; i <= 0x23; i++){
        i2c5_write_registers(0xD0, (unsigned char[2]){i, 0x00}, 2);
    }
    i2c5_write_registers(0xD0, (unsigned char[2]){0x24, 0x40}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x25, 0x8C}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x26, 0x02}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x27, 0x88}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x28, 0x0C}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x29, 0x0A}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x2A, 0x81}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x64, 0x01}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x67, 0x03}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x01, 0x80}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x34, 0x04}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x64, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x64, 0x01}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x20}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x34, 0x13}, 2);
    for(i = 0; i < 5; i++){
        acc_buffer[i].x = 0;
        acc_buffer[i].y = 0;
        acc_buffer[i].z = 0;
        gyro_buffer[i].x = 0;
        gyro_buffer[i].y = 0;
        gyro_buffer[i].z = 0;
    }
}

void GetAcc(){
    unsigned char temp[6];
    unsigned char i;
    if(BUFFER_SIZE > 0){
        for(i = (BUFFER_SIZE - 1); i >= 1; i--){
            acc_buffer[i].x = acc_buffer[i - 1].x;
            acc_buffer[i].y = acc_buffer[i - 1].y;
            acc_buffer[i].z = acc_buffer[i - 1].z;
        }
    }
    i2c5_read_registers(0xD0, 0x3B, temp, 6);
    acc.y = (signed short)(temp[0] << 8 | temp[1]);
    acc.x = (signed short)(temp[2] << 8 | temp[3]) * (-1);
    acc.z = (signed short)(temp[4] << 8 | temp[5]) * (-1);
    if(BUFFER_SIZE > 0){
        acc_buffer[0].x = acc.x;
        acc_buffer[0].y = acc.y;
        acc_buffer[0].z = acc.z;
        for(i = 1; i < BUFFER_SIZE; i++){
            acc.x += acc_buffer[i].x;
            acc.y += acc_buffer[i].y;
            acc.z += acc_buffer[i].z;
        }
        acc.x /= BUFFER_SIZE;
        acc.y /= BUFFER_SIZE;
        acc.z /= BUFFER_SIZE;
    }
}

void GetGyro(){
    unsigned char temp[6];
    unsigned char i;
    if(BUFFER_SIZE > 0){
        for(i = (BUFFER_SIZE - 1); i >= 1; i--){
            gyro_buffer[i].x = gyro_buffer[i - 1].x;
            gyro_buffer[i].y = gyro_buffer[i - 1].y;
            gyro_buffer[i].z = gyro_buffer[i - 1].z;
        }
    }
    i2c5_read_registers(0xD0, 0x43, temp, 6);
    gyro.y = (signed short)(temp[0] << 8 | temp[1]);
    gyro.x = (signed short)(temp[2] << 8 | temp[3]);
    gyro.z = (signed short)(temp[4] << 8 | temp[5]);
    if(BUFFER_SIZE > 0){
        gyro_buffer[0].x = gyro.x;
        gyro_buffer[0].y = gyro.y;
        gyro_buffer[0].z = gyro.z;
        for(i = 1; i < BUFFER_SIZE; i++){
            gyro.x += gyro_buffer[i].x;
            gyro.y += gyro_buffer[i].y;
            gyro.z += gyro_buffer[i].z;
        }
        gyro.x /= BUFFER_SIZE;
        gyro.y /= BUFFER_SIZE;
        gyro.z /= BUFFER_SIZE;
    }
    gyro.x = (gyro.x - GYRO_X_OFFSET) / GYRO_X_GAIN;
    gyro.y = (gyro.y - GYRO_Y_OFFSET) / GYRO_Y_GAIN;
    gyro.z = (gyro.z - GYRO_Z_OFFSET) / GYRO_Z_GAIN;
}

//-----------------------------------------HMC5883---------------------------------

void HMC5883Init(){
    unsigned char i;
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x37, 0x02}, 2);
    for(i = 0; i < 5; i++){
        compass_buffer[i].x = 0;
        compass_buffer[i].y = 0;
        compass_buffer[i].z = 0;
    }
    i2c5_write_registers(0x3C, (unsigned char[2]){0, 0x14}, 2);
    i2c5_write_registers(0x3C, (unsigned char[2]){1, 0x20}, 2);
    i2c5_write_registers(0x3C, (unsigned char[2]){2, 0x00}, 2);
    
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x20}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x37, 0x00}, 2);
    for(i = 0; i < 5; i++){
        compass_buffer[i].x = 0;
        compass_buffer[i].y = 0;
        compass_buffer[i].z = 0;
    }
}

void GetCompass(){
    unsigned char temp[6];
    unsigned char i;
    if(BUFFER_SIZE > 0){
        for(i = (BUFFER_SIZE - 1); i >= 1; i--){
            compass_buffer[i].x = compass_buffer[i - 1].x;
            compass_buffer[i].y = compass_buffer[i - 1].y;
            compass_buffer[i].z = compass_buffer[i - 1].z;
        }
    }
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x37, 0x02}, 2);
    i2c5_read_registers(0x3C, 0x03, temp, 6);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x20}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x37, 0x00}, 2);
    compass.y = (signed short)(temp[0] << 8 | temp[1]);//y
    compass.z = (signed short)(temp[2] << 8 | temp[3]);
    compass.x = (signed short)(temp[4] << 8 | temp[5]);//x
    if(BUFFER_SIZE > 0){
        compass_buffer[0].x = compass.x;
        compass_buffer[0].y = compass.y;
        compass_buffer[0].z = compass.z;
        for(i = 1; i < BUFFER_SIZE; i++){
            compass.x += compass_buffer[i].x;
            compass.y += compass_buffer[i].y;
            compass.z += compass_buffer[i].z;
        }
        compass.x /= BUFFER_SIZE;
        compass.y /= BUFFER_SIZE;
        compass.z /= BUFFER_SIZE;
    }
    compass.x = (compass.x - COMPASS_X_OFFSET) * COMPASS_X_GAIN;
    compass.y = (compass.y - COMPASS_Y_OFFSET) * COMPASS_Y_GAIN;
    compass.z = (compass.z - COMPASS_Z_OFFSET) * COMPASS_Z_GAIN;
}

//-----------------------------------------BMP180---------------------------------

void BMP180Init(){
    unsigned char temp[22];
    i2c5_read_registers(0xEE, 0xAA, temp, 22);
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
}

void GetRawTemperature(){
    float X1, X2;
    unsigned char t[2];
    i2c5_read_registers(0xEE, 0xF6, t, 2);
    UT = t[0] << 8 | t[1];
    X1 = ((float)UT - (float)ac6) * ((float)ac5 / 32768.0f);
    X2 = ((float)(mc * 2048.0f)) / (X1 + (float)md);
    B5 = X1 + X2;
}

float get_temperature() {
    float temp;
    temp = (B5 + 8) / 16.0f;
    temp /= 10;
    return temp;
}

float GetAltitude(){
    long int UP, p;
    float altitude, X1, X2, X3, B3, B6;
    unsigned long int B4, B7;
    unsigned char t[3];
    
    i2c5_read_registers(0xEE, 0xF6, t, 3);
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

void GetRawIMU(){
    unsigned char temp[6];
    i2c5_read_registers(0xD0, 0x3B, temp, 6);
    acc.y = (signed short)(temp[0] << 8 | temp[1]);
    acc.x = (signed short)(temp[2] << 8 | temp[3]) * (-1);
    acc.z = (signed short)(temp[4] << 8 | temp[5]) * (-1);
    i2c5_read_registers(0xD0, 0x43, temp, 6);
    gyro.y = (signed short)(temp[0] << 8 | temp[1]);
    gyro.x = (signed short)(temp[2] << 8 | temp[3]);
    gyro.z = (signed short)(temp[4] << 8 | temp[5]);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x37, 0x02}, 2);
    i2c5_read_registers(0x3C, 0x03, temp, 6);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x20}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x37, 0x00}, 2);
    compass.y = (signed short)(temp[0] << 8 | temp[1]);//y
    compass.z = (signed short)(temp[2] << 8 | temp[3]);
    compass.x = (signed short)(temp[4] << 8 | temp[5]);//x
}
