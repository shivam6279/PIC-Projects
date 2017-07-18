#ifndef _MPU9150_H_
#define _MPU9150_H_

#define MAX_BUFFER_SIZE 5

#define gyro_sensitivity 95

typedef struct {
    double x,y,z;
} XYZ;

void MPU9150_write(unsigned char address, unsigned char data);
void MPU9150_compass_write(unsigned char address, unsigned char data);
void MPU9150_init();
void MPU9150_get_acc();
void MPU9150_get_gyro();
void MPU9150_get_compass();
float MPU9150_get_acc_x_angle();
float MPU9150_get_acc_y_angle();
float MPU9150_get_heading();

XYZ acc, acc_buffer[MAX_BUFFER_SIZE];
XYZ gyro, gyro_buffer[MAX_BUFFER_SIZE];
XYZ compass, compass_buffer[MAX_BUFFER_SIZE];
XYZ gyro_offset;
XUZ compass_offset;
XYZ compass_gain;

unsigned char buffer_size;

void MPU9150_write(unsigned char address, unsigned char data){
    i2c_start();
    i2c_send(0xD0);
    i2c_send(address);
    i2c_send(data);
    i2c_stop();
}

void MPU9150_compass_write(unsigned char address, unsigned char data){
    i2c_start();
    i2c_send(0x18);
    i2c_send(address);
    i2c_send(data);
    i2c_stop();
}

void MPU9150_init(XYZ a, XYZ b, XYZ c){
    unsigned char i;
    MPU9150_write(0x6B, 0);
    MPU9150_compass_write(0x0A, 0x00);
    MPU9150_compass_write(0x0A, 0x0F);
    MPU9150_compass_write(0x0A, 0x00);
    MPU9150_write(0x19, 0x07);
    MPU9150_write(0x1A, 0x03);
    MPU9150_write(0x1B, 0x00);
    MPU9150_write(0x1C, 0x00);
    for(i = 0x1D; i <= 0x23; i++){
        MPU9150_write(i, 0x00);
    }
    MPU9150_write(0x24, 0x40);
    MPU9150_write(0x25, 0x8C);
    MPU9150_write(0x26, 0x02);
    MPU9150_write(0x27, 0x88);
    MPU9150_write(0x28, 0x0C);
    MPU9150_write(0x29, 0x0A);
    MPU9150_write(0x2A, 0x81);
    MPU9150_write(0x64, 0x01);
    MPU9150_write(0x67, 0x03);
    MPU9150_write(0x01, 0x80);
    MPU9150_write(0x34, 0x04);
    MPU9150_write(0x64, 0x00);
    MPU9150_write(0x6A, 0x00);
    MPU9150_write(0x64, 0x01);
    MPU9150_write(0x6A, 0x20);
    MPU9150_write(0x34, 0x13);
    gyro_offset.x = a.x;
    gyro_offset.y = a.y;
    gyro_offset.z = a.z;
    compass_offset.x = b.x;
    compass_offset.y = b.y;
    compass_offset.z = b.z;
    compass_gain.x = c.x;
    compass_gain.y = c.y;
    compass_gain.z = c.z;
    for(i = 0; i < 5; i++){
        acc_buffer[i].x = 0;
        acc_buffer[i].y = 0;
        acc_buffer[i].z = 0;
        gyro_buffer[i].x = 0;
        gyro_buffer[i].y = 0;
        gyro_buffer[i].z = 0;
        compass_buffer[i].x = 0;
        compass_buffer[i].y = 0;
        compass_buffer[i].z = 0;
    }
}

void MPU9150_get_acc(){
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

void MPU9150_get_gyro(){
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

void MPU9150_get_compass(){
    unsigned char i;
    if(buffer_size > 0){
        for(i = (buffer_size - 1); i >= 1; i--){
            compass_buffer[i].x = compass_buffer[i - 1].x;
            compass_buffer[i].y = compass_buffer[i - 1].y;
            compass_buffer[i].z = compass_buffer[i - 1].z;
        }
    }
    i2c_start();
    i2c_send(0xD0);
    i2c_send(0x4A);
    i2c_restart();
    i2c_send(0xD1);
    compass.y = i2c_read() << 8;
    i2c_ack();
    compass.y += i2c_read();
    i2c_ack();
    compass.x = i2c_read() << 8;
    i2c_ack();
    compass.x += i2c_read();
    i2c_ack();
    compass.z = i2c_read() << 8;
    i2c_ack();
    compass.z += i2c_read();
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
    compass.x = (compass.x - compass_offset.x) * compass_gain.x;
    compass.y = (compass.y - compass_offset.y) * compass_gain.y;
    compass.z = (compass_offset.z - compass.z) * compass_gain.z;
}

 float MPU9150_get_acc_x_angle(){
    return (64 * atan2(acc.y, sqrt(acc.z * acc.z + acc.x * acc.x)));
}

float MPU9150_get_acc_y_angle(){
    return ((-64) * atan2(acc.x, sqrt(acc.z * acc.z + acc.y * acc.y)));
}

float MPU9150_get_heading(){
    return atan2(-compass.y, compass.x);
}

#endif
