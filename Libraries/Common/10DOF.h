#ifndef _10DOF_H_
#define _10DOF_H_

#define MAX_BUFFER_SIZE 5

#define gyro_sensitivity 91.2
    
#define gyro_x_offset -180.5
#define gyro_y_offset -138.5
#define gyro_z_offset -90.25

#define compass_x_offset 0//-252.5
#define compass_y_offset 0//615
#define compass_z_offset 0//-310

#define compass_x_gain 1//1.652892562
#define compass_y_gain 1//1.265822784
#define compass_z_gain 1//1.612903225

typedef struct {
    double x,y,z;
} XYZ;

void MPU6050_write(unsigned char address, unsigned char data);
void MPU6050_init();
void HMC5883_init();
void calibrate_gyros();
void get_acc();
void get_gyro();
void get_compass();
float get_acc_x_angle();
float get_acc_y_angle();
float get_compensated_heading(float sinx, float cosx, float siny, float cosy);

XYZ acc, acc_buffer[MAX_BUFFER_SIZE];
XYZ gyro, gyro_buffer[MAX_BUFFER_SIZE];
XYZ compass, compass_buffer[MAX_BUFFER_SIZE];

unsigned char buffer_size;

void MPU6050_write(unsigned char address, unsigned char data){
    i2c_start();
    i2c_send(0xD0);
    i2c_send(address);
    i2c_send(data);
    i2c_stop();
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
    MPU6050_write(0x6B, 0);
    
    i2c_start();
    i2c_send(0x3C);
    i2c_send(0x00);
    i2c_send(0x14);
    i2c_stop();
    i2c_start();
    i2c_send(0x3C);
    i2c_send(0x01);
    i2c_send(0x20);
    i2c_stop();
    i2c_start();
    i2c_send(0x3C);
    i2c_send(0x02);
    i2c_send(0x00);
    i2c_stop();
    
    MPU6050_write(0x6A, 0x20);
    MPU6050_write(0x37, 0);
    MPU6050_write(0x25, 0x9E);
    MPU6050_write(0x26, 0x03);
    MPU6050_write(0x27, 0x86);
    for(i = 0; i < 5; i++){
        compass_buffer[i].x = 0;
        compass_buffer[i].y = 0;
        compass_buffer[i].z = 0;
    }
}

void get_acc(){
    unsigned char i;
    if(buffer_size > 0){
        for(i = (buffer_size - 1); i >= 1; i--){
            acc_buffer[i].x = acc_buffer[i - 1].x;
            acc_buffer[i].y = acc_buffer[i - 1].y;
            acc_buffer[i].z = acc_buffer[i - 1].z;
        }
    }
    i2c_start();
    i2c_send(0xD0);
    i2c_send(0x3B);
    i2c_restart();
    i2c_send(0xD1);
    acc.x = i2c_read() << 8;
    i2c_ack();
    acc.x += i2c_read();
    i2c_ack();
    acc.y = i2c_read() << 8;
    i2c_ack();
    acc.y += i2c_read();
    i2c_ack();
    acc.z = i2c_read() << 8;
    i2c_ack();
    acc.z += i2c_read();
    i2c_nak();
    i2c_stop();
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
    unsigned char i;
    if(buffer_size > 0){
        for(i = (buffer_size - 1); i >= 1; i--){
            gyro_buffer[i].x = gyro_buffer[i - 1].x;
            gyro_buffer[i].y = gyro_buffer[i - 1].y;
            gyro_buffer[i].z = gyro_buffer[i - 1].z;
        }
    }
    i2c_start();
    i2c_send(0xD0);
    i2c_send(0x43);
    i2c_restart();
    i2c_send(0xD1);
    gyro.x = i2c_read() << 8;
    i2c_ack();
    gyro.x += i2c_read();
    i2c_ack();
    gyro.y = i2c_read() << 8;
    i2c_ack();
    gyro.y += i2c_read();
    i2c_ack();
    gyro.z = i2c_read() << 8;
    i2c_ack();
    gyro.z += i2c_read();
    i2c_nak();
    i2c_stop();
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
    gyro.y = (gyro.y - gyro_y_offset) / gyro_sensitivity;
    gyro.z = (gyro.z - gyro_z_offset) / gyro_sensitivity;
}

void get_compass(){
    int i;
    if(buffer_size > 0){
        for(i = (buffer_size - 1); i >= 1; i--){
            compass_buffer[i].x = compass_buffer[i - 1].x;
            compass_buffer[i].y = compass_buffer[i - 1].y;
            compass_buffer[i].z = compass_buffer[i - 1].z;
        }
    }
    i2c_start();
    i2c_send(0xD0);
    i2c_send(0x49);
    i2c_restart();
    i2c_send(0xD1);
    compass.y = i2c_read() << 8;
    i2c_ack();
    compass.y += i2c_read();
    i2c_ack();
    compass.z = i2c_read() << 8;
    i2c_ack();
    compass.z += i2c_read();
    i2c_ack();
    compass.x = i2c_read() << 8;
    i2c_ack();
    compass.x += i2c_read();
    i2c_nak();
    i2c_stop();
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
    compass.x = (compass.x - compass_x_offset) * compass_x_gain;
    compass.y = (compass_y_offset - compass.y) * compass_y_gain;
    compass.z = (compass.z - compass_z_offset) * compass_z_gain;
}

 float get_acc_x_angle(){
    return (64 * atan2(acc.y, sqrt(acc.z * acc.z + acc.x * acc.x)));
}

float get_acc_y_angle(){
    return ((-64) * atan2(acc.x, sqrt(acc.z * acc.z + acc.y * acc.y)));
}

float get_compensated_heading(float sinx, float cosx, float siny, float cosy){
    return atan2((compass.y * cosy - compass.z * siny), (compass.x * cosx + compass.y * sinx * siny + compass.z * cosy * sinx));
}

#endif
