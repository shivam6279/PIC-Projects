#ifndef _MPU6050_H_
#define _MPU6050_H_

#define gyro_sensitivity 32

void MPU6050_write(unsigned char address, unsigned char data);
void MPU6050_init();
void calibrate_gyros();
void get_acc();
void get_gyro();
float get_acc_x_angle();
float get_acc_y_angle();

double gyro_x_offset, gyro_y_offset, gyro_z_offset, gyro_x,gyro_y, gyro_z, acc_x, acc_y, acc_z;

void MPU6050_write(unsigned char address, unsigned char data){
    i2c_start();
    i2c_send(0xD0);
    i2c_send(address);
    i2c_send(data);
    i2c_stop();
}

void MPU6050_init(){
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
}

void calibrate_gyros(){
    int i;
    double x_sum = 0,y_sum = 0,z_sum = 0;
    for(i = 0; i < 500; i++){
        i2c_start();
        i2c_send(0xD0);
        i2c_send(0x41);
        i2c_restart();
        i2c_send(0xD1);
        x_sum += 256 * i2c_read();
        i2c_ack();
        x_sum += i2c_read();
        i2c_ack();
        y_sum += 256 * i2c_read();
        i2c_ack();
        y_sum += i2c_read();
        i2c_ack();
        z_sum += 256 * i2c_read();
        i2c_ack();
        z_sum += i2c_read();
        i2c_nak();
        i2c_stop();
 	    __delay_ms(1);
    }
    gyro_x_offset = (float)(x_sum / 500);
    gyro_y_offset = (float)(y_sum / 500);
    gyro_z_offset = (float)(z_sum / 500);
}

void get_acc(){
    i2c_start();
    i2c_send(0xD0);
    i2c_send(0x3B);
    i2c_restart();
    i2c_send(0xD1);
    acc_x= 256 * i2c_read();
    i2c_ack();
    acc_x += i2c_read();
    i2c_ack();
    acc_y = 256 * i2c_read();
     i2c_ack();
    acc_y += i2c_read();
    i2c_ack();
    acc_z = 256 * i2c_read();
    i2c_ack();
    acc_z += i2c_read();
    i2c_nak();
    i2c_stop();
}

void get_gyro(){
    i2c_start();
    i2c_send(0xD0);
    i2c_send(0x43);
    i2c_restart();
    i2c_send(0xD1);
    gyro_x= 256 * i2c_read();
    i2c_ack();
    gyro_x += i2c_read();
    i2c_ack();
    gyro_y = 256 * i2c_read();
     i2c_ack();
    gyro_y += i2c_read();
    i2c_ack();
    gyro_z = 256 * i2c_read();
    i2c_ack();
    gyro_z += i2c_read();
    i2c_nak();
    i2c_stop();
    gyro_x = (gyro_x - gyro_x_offset) / gyro_sensitivity;
    gyro_y = (gyro_y - gyro_y_offset) / gyro_sensitivity;
    gyro_z = (gyro_z - gyro_z_offset) / gyro_sensitivity;
}

float get_acc_x_angle(){
    return (64 * atan(acc_y / sqrt(acc_z * acc_z + acc_x * acc_x)));
}

float get_acc_y_angle(){
    return (64 * atan(acc_x / sqrt(acc_z * acc_z + acc_y * acc_y)));
}

#endif
