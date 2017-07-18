#ifndef _HMC5883_H_
#define _HMC5883_H_

/*
Digital compass HMC5883
Requires I2C.h and math.h
*/

void HMC5883_init();
void compass_read();
float get_heading();

double comp_x, comp_y, comp_z;

void HMC5883_init(){
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
}

void compass_read(){
    i2c_start();
    i2c_send(0x3C);
    i2c_send(0x03);
    i2c_restart();
    i2c_send(0x3D);
    comp_x = 256 * i2c_read();
    i2c_ack();
    comp_x += i2c_read();
    i2c_ack();
    comp_z = 256 * i2c_read();
    i2c_ack();
    comp_z += i2c_read();
    i2c_ack();
    comp_y = 256 * i2c_read();
    i2c_ack();
    comp_y += i2c_read();
    i2c_nak();
    i2c_stop();
}

float get_heading(){
    return ((atan2(comp_y, comp_x) * 58.7) + 184);
}

#endif

