#ifndef _HMC5883_H_
#define _HMC5883_H_

/*
Digital compass HMC5883
Requires I2C.h and math.h
*/

#define compass_x_offset -2.5
#define compass_y_offset -115
#define compass_z_offset -60

#define compass_x_gain 1.41844
#define compass_y_gain 1.49254
#define compass_z_gain 1.5625

void HMC5883_init();
void get_compass();
float get_heading();
float get_compensated_heading(float sinx, float cosx, float siny, float cosy);

XYZ compass, compass_buffer[MAX_BUFFER_SIZE];

void HMC5883_init(){
    unsigned char i;
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
    for(i = 0; i < 5; i++){
        compass_buffer[i].x = 0;
        compass_buffer[i].y = 0;
        compass_buffer[i].z = 0;
    }
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
    i2c_send(0x3C);
    i2c_send(0x03);
    i2c_restart();
    i2c_send(0x3D);
    compass.x = i2c_read() << 8;
    i2c_ack();
    compass.x += i2c_read();
    i2c_ack();
    compass.z = i2c_read() << 8;
    i2c_ack();
    compass.z += i2c_read();
    i2c_ack();
    compass.y = i2c_read() << 8;
    i2c_ack();
    compass.y += i2c_read();
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
    compass.y = (compass.y - compass_y_offset) * compass_y_gain;
    compass.z = (compass.z - compass_z_offset) * compass_z_gain;
}

float get_heading(){
    return atan2(compass.y, compass.x);
}

float get_compensated_heading(float sinx, float cosx, float siny, float cosy){
    return atan2((compass.y * cosy - compass.z * siny), (compass.x * cosx + compass.y * sinx * siny + compass.z * cosy * sinx));
}

#endif

