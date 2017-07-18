#ifndef _MPU6050_H_
#define _MPU6050_H_

#define gyro_sensitivity 32

typedef struct {
    double x,y,z;
} XYZ;

void MPU6050_write(unsigned char address, unsigned char data);
void MPU6050_init();
void MPU6050_get_acc();
void MPU6050_get_gyro();
float MPU6050_get_acc_x_angle();
float MPU6050_get_acc_y_angle();

XYZ acc, gyro, gyro_offset;

void MPU6050_write(unsigned char address, unsigned char data){
    i2c_start();
    i2c_send(0xD0);
    i2c_send(address);
    i2c_send(data);
    i2c_stop();
}

void MPU6050_init(XYZ a){
    int i;
    MPU6050_write(0x19, 0x07);
    MPU6050_write(0x1A, 0x00);
    MPU6050_write(0x1B, 0x08);
    for(i = 0x1C; i <= 0x35; i++){
        MPU6050_write(i, 0x00);
    }
    MPU6050_write(0x37, 0x00);
    MPU6050_write(0x38, 0x00);
    for(i = 0x63; i <= 0x6C; i++){
        MPU6050_write(i, 0x00);
    }
    MPU6050_write(0x74, 0x00);
    gyro_offset.x = a.x;
    gyro_offset.y = a.y;
    gyro_offset.z = a.z;
}

void MPU6050_get_acc(){
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
}

void MPU6050_get_gyro(){
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
    gyro.x = (gyro.x - gyro_offset.x) / gyro_sensitivity;
    gyro.y = (gyro.y - gyro_offset.y) / gyro_sensitivity;
    gyro.z = (gyro.z - gyro_offset.z) / gyro_sensitivity;
}

float MPU6050_get_acc_x_angle(){
    return (64 * atan(acc.y / sqrt(acc.z * acc.z + acc.x * acc.x)));
}

float MPU6050_get_acc_y_angle(){
    return (64 * atan(acc.x / sqrt(acc.z * acc.z + acc.y * acc.y)));
}

#endif
