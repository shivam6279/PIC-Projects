#include "PWMDriver.h"
#include "bitbang_i2c.h" 

void pwm_driver_init(float freq){
    //Highest frequency 1600hz
    unsigned int t = 20000;
    unsigned char prescale;
    float prescaleval;
    freq *= 0.9;
    prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    prescale = (int)(prescaleval + 0.5);
    i2c3_write_registers(128, (unsigned char[2]){0, 16}, 2);
    i2c3_write_registers(128, (unsigned char[2]){0xFE, prescale}, 2);
    i2c3_write_registers(128, (unsigned char[2]){0, 0}, 2);
    while(t-- > 0);
    i2c3_write_registers(128, (unsigned char[2]){0, 0xA1}, 2);
}

void set_pwm(int num, int on, int off){
    i2c3_write_registers(128, (unsigned char[5]){(0x06 + 4 * num), (on & 0xFF), ((on >> 8) & 0x1F), (off & 0xFF), ((off >> 8) & 0x1F)}, 5);
}

void write_pwm(int num, int val){
    if (val == 0) {
        set_pwm(num, 0, 4096);
    }
    else if (val >= 4095) {
        set_pwm(num, 4096, 0);
    }
    else {
        set_pwm(num, 0, val);
    }
}