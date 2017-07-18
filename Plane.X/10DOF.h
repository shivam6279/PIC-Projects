#ifndef _10DOF_H_
#define _10DOF_H_

#define MAX_BUFFER_SIZE 5

#define gyro_sensitivity 98
    
#define gyro_x_offset 0
#define gyro_y_offset 0
#define gyro_z_offset 0

#define compass_x_offset -1900
#define compass_y_offset -1025
#define compass_z_offset -850

#define compass_x_gain 1.052631579
#define compass_y_gain 1.126760563
#define compass_z_gain 1.081081081

typedef struct {
    double x,y,z;
} XYZ;

void LSM303D_write(unsigned char address, unsigned char data);
void LSM303D_init();
//void HMC5883_init();
void get_acc();
//void get_gyro();
void get_compass();
float get_compensated_heading(float sinx, float cosx, float siny, float cosy);

XYZ acc;
//XYZ gyro;
XYZ compass;

unsigned char buffer_size;

void LSM303D_write(unsigned char address, unsigned char data){
    i2c_start();
    i2c_send(0x3A);
    i2c_send(address);
    i2c_send(data);
    i2c_stop();
}

void LSM303D_init(){
    LSM303D_write(0x20, 0x57);
    LSM303D_write(0x21, 0);
    LSM303D_write(0x24, 0x64);
    LSM303D_write(0x25, 0x20);
    LSM303D_write(0x26, 0);
}

void get_acc(){
    i2c_start();
    i2c_send(0x3A);
    i2c_send(0x28 | (1 << 7));
    i2c_restart();
    i2c_send(0x3B);
    acc.y = i2c_read();
    i2c_ack();
    acc.y += i2c_read() << 8;
    i2c_ack();
    acc.x = i2c_read();
    i2c_ack();
    acc.x += i2c_read() << 8;
    i2c_ack();
    acc.z = i2c_read();
    i2c_ack();
    acc.z += i2c_read() << 8;
    i2c_nak();
    i2c_stop();
}

void get_compass(){
    i2c_start();
    i2c_send(0x3A);
    i2c_send(0x08 | (1 << 7));
    i2c_restart();
    i2c_send(0x3B);
    compass.x = i2c_read();
    i2c_ack();
    compass.x += i2c_read() << 8;
    i2c_ack();
    compass.y = i2c_read();
    i2c_ack();
    compass.y += i2c_read() << 8;
    i2c_ack();
    compass.z = i2c_read();
    i2c_ack();
    compass.z += i2c_read() << 8;
    i2c_nak();
    i2c_stop();
    compass.x = (compass.x - compass_x_offset) / compass_x_gain;
    compass.y = (compass.y - compass_y_offset) / compass_y_gain;
    compass.z = (compass.z - compass_z_offset) / compass_z_gain;
}

float get_compensated_heading(float sinx, float cosx, float siny, float cosy){
    return atan2((compass.y * cosy - compass.z * siny), (compass.x * cosx + compass.y * sinx * siny + compass.z * cosy * sinx));
}

#endif
