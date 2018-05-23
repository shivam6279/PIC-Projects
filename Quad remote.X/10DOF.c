#include "10DOF.h"
#include "bitbang_i2c.h"
#include <math.h>

void MPU6050_write(unsigned char address, unsigned char data){
    i2c5_write_registers(0xD0, (unsigned char[2]){address, data}, 2);
}

void MPU6050_init(){
    unsigned char i;
    MPU6050_write(0x6B, 0);
    MPU6050_write(0x19, 0x07);
    MPU6050_write(0x1A, 0x03);
    MPU6050_write(0x1B, 0x00);
    MPU6050_write(0x1C, 0x00);
    for(i = 0x1D; i <= 0x23; i++){
        MPU6050_write(i, 0x00);
    }
    MPU6050_write(0x24, 0x40);
    MPU6050_write(0x25, 0x8C);
    MPU6050_write(0x26, 0x02);
    MPU6050_write(0x27, 0x88);
    MPU6050_write(0x28, 0x0C);
    MPU6050_write(0x29, 0x0A);
    MPU6050_write(0x2A, 0x81);
    MPU6050_write(0x64, 0x01);
    MPU6050_write(0x67, 0x03);
    MPU6050_write(0x01, 0x80);
    MPU6050_write(0x34, 0x04);
    MPU6050_write(0x64, 0x00);
    MPU6050_write(0x6A, 0x00);
    MPU6050_write(0x64, 0x01);
    MPU6050_write(0x6A, 0x20);
    MPU6050_write(0x34, 0x13);
    for(i = 0; i < 5; i++){
        acc_buffer[i].x = 0;
        acc_buffer[i].y = 0;
        acc_buffer[i].z = 0;
        gyro_buffer[i].x = 0;
        gyro_buffer[i].y = 0;
        gyro_buffer[i].z = 0;
    }
}

void HMC5883_init(){
    unsigned char i;
    MPU6050_write(0x6A, 0);
    MPU6050_write(0x37, 2);
    for(i = 0; i < 5; i++){
        compass_buffer[i].x = 0;
        compass_buffer[i].y = 0;
        compass_buffer[i].z = 0;
    }
    i2c5_write_registers(0x3C, (unsigned char[2]){0, 0x14}, 2);
    i2c5_write_registers(0x3C, (unsigned char[2]){1, 0x20}, 2);
    i2c5_write_registers(0x3C, (unsigned char[2]){2, 0x00}, 2);
    
    MPU6050_write(0x6A, 0x20);
    MPU6050_write(0x37, 0);
    //MPU6050_write(0x25, 0x9E);
    //MPU6050_write(0x26, 0x03);
    //MPU6050_write(0x27, 0x86);
    for(i = 0; i < 5; i++){
        compass_buffer[i].x = 0;
        compass_buffer[i].y = 0;
        compass_buffer[i].z = 0;
    }
}

void get_acc(){
    unsigned char temp[6];
    unsigned char i;
    if(buffer_size > 0){
        for(i = (buffer_size - 1); i >= 1; i--){
            acc_buffer[i].x = acc_buffer[i - 1].x;
            acc_buffer[i].y = acc_buffer[i - 1].y;
            acc_buffer[i].z = acc_buffer[i - 1].z;
        }
    }
    i2c5_read_registers(0xD0, 0x3B, temp, 6);
    acc.y = (signed short)(temp[0] << 8 | temp[1]);
    acc.x = (signed short)(temp[2] << 8 | temp[3]) * (-1);
    acc.z = (signed short)(temp[4] << 8 | temp[5]) * (-1);
    if(buffer_size > 0){
        acc_buffer[0].x = acc.x;
        acc_buffer[0].y = acc.y;
        acc_buffer[0].z = acc.z;
        for(i = 1; i < buffer_size; i++){
            acc.x += acc_buffer[i].x;
            acc.y += acc_buffer[i].y;
            acc.z += acc_buffer[i].z;
        }
        acc.x /= buffer_size;
        acc.y /= buffer_size;
        acc.z /= buffer_size;
    }
}

void get_gyro(){
    unsigned char temp[6];
    unsigned char i;
    if(buffer_size > 0){
        for(i = (buffer_size - 1); i >= 1; i--){
            gyro_buffer[i].x = gyro_buffer[i - 1].x;
            gyro_buffer[i].y = gyro_buffer[i - 1].y;
            gyro_buffer[i].z = gyro_buffer[i - 1].z;
        }
    }
    i2c5_read_registers(0xD0, 0x43, temp, 6);
    gyro.y = (signed short)(temp[0] << 8 | temp[1]);
    gyro.x = (signed short)(temp[2] << 8 | temp[3]);
    gyro.z = (signed short)(temp[4] << 8 | temp[5]);
    if(buffer_size > 0){
        gyro_buffer[0].x = gyro.x;
        gyro_buffer[0].y = gyro.y;
        gyro_buffer[0].z = gyro.z;
        for(i = 1; i < buffer_size; i++){
            gyro.x += gyro_buffer[i].x;
            gyro.y += gyro_buffer[i].y;
            gyro.z += gyro_buffer[i].z;
        }
        gyro.x /= buffer_size;
        gyro.y /= buffer_size;
        gyro.z /= buffer_size;
    }
    gyro.x = (gyro.x - gyro_x_offset) / gyro_sensitivity;
    gyro.y = (gyro_y_offset - gyro.y) / gyro_sensitivity;
    gyro.z = (gyro.z - gyro_z_offset) / gyro_sensitivity;
}

void get_compass(){
    unsigned char temp[6];
    unsigned char i;
    if(buffer_size > 0){
        for(i = (buffer_size - 1); i >= 1; i--){
            compass_buffer[i].x = compass_buffer[i - 1].x;
            compass_buffer[i].y = compass_buffer[i - 1].y;
            compass_buffer[i].z = compass_buffer[i - 1].z;
        }
    }
    MPU6050_write(0x6A, 0);
    MPU6050_write(0x37, 2);
    i2c5_read_registers(0x3C, 0x03, temp, 6);
    MPU6050_write(0x6A, 0x20);
    MPU6050_write(0x37, 0);
    compass.y = (signed short)(temp[0] << 8 | temp[1]);//y
    compass.z = (signed short)(temp[2] << 8 | temp[3]);
    compass.x = (signed short)(temp[4] << 8 | temp[5]);//x
    if(buffer_size > 0){
        compass_buffer[0].x = compass.x;
        compass_buffer[0].y = compass.y;
        compass_buffer[0].z = compass.z;
        for(i = 1; i < buffer_size; i++){
            compass.x += compass_buffer[i].x;
            compass.y += compass_buffer[i].y;
            compass.z += compass_buffer[i].z;
        }
        compass.x /= buffer_size;
        compass.y /= buffer_size;
        compass.z /= buffer_size;
    }
    compass.x = (compass.x - compass_x_offset) * compass_x_gain;//x
    compass.y = (compass.y - compass_y_offset) * compass_y_gain;//y
    compass.z = (compass.z - compass_z_offset) * compass_z_gain;
}

 float get_acc_x_angle(){
    return (atan2(acc.y, sqrt(acc.z * acc.z + acc.x * acc.x)));
}

float get_acc_y_angle(){
    return (atan2(acc.x, sqrt(acc.z * acc.z + acc.y * acc.y)));
}

float get_compensated_heading(float sinx, float cosx, float siny, float cosy){
    return atan2((compass.z * siny - compass.y * cosy), (compass.x * cosx + compass.y * sinx * siny + compass.z * cosy * sinx));
}
