#ifndef _PWMDriver_H_
#define _PWMDriver_H_

/*
I2c PWM driver. Frequency up to 1600hz
Requires I2C.h and xc.h
*/

void pwm_driver_init(float freq);
void set_pwm(int num, int on, int off);
void write_pwm(int num, int val);

void pwm_driver_init(float freq){
    //Highest frequency 1600hz
    int prescale;
    float prescaleval;
    i2c_start();
    i2c_send(128);
    i2c_send(0);
    i2c_send(16);
    i2c_stop();
    freq *= 0.9;
    prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    prescale = (int)(prescaleval + 0.5);
    i2c_start();
    i2c_send(128);
    i2c_send(0xFE);
    i2c_send(prescale);
    i2c_stop();
    i2c_start();
    i2c_send(128);
    i2c_send(0);
    i2c_send(0);
    i2c_stop();
    __delay_ms(5);
    i2c_start();
    i2c_send(128);
    i2c_send(0);
    i2c_send(0xA1);
    i2c_stop();
}

void set_pwm(int num, int on, int off){
    i2c_start();
    i2c_send(128);
    i2c_send(0x06 + 4 * num);
    i2c_send((on & 0xFF));
    i2c_send(((on >> 8) & 0x1F));
    i2c_send((off & 0xFF));
    i2c_send(((off >> 8) & 0x1F));
    i2c_stop();
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

#endif