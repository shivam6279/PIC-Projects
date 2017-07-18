#ifndef _PWMDriver_H_
#define _PWMDriver_H_

void pwm_driver_init(float freq);
void set_pwm(int num, int on, int off);
void write_pwm(int num, int val);
void write_pwm_four(int num, int a, int b, int c, int d);

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

void write_pwm_four(int num, int a, int b, int c, int d){
    int off, on;
    i2c_start();
    i2c_send(128);
    i2c_send(0x06 + 4 * num);
    if (a == 0){
        on = 0;
        off = 4096;
    }
    else if (a >= 4095) {
        on = 4096;
        off = 0;
    }
    else {
        on = 0;
        off = a;
    }
    i2c_send((on & 0xFF));
    i2c_send(((on >> 8) & 0x1F));
    i2c_send((off & 0xFF));
    i2c_send(((off >> 8) & 0x1F));
    if (b == 0){
        on = 0;
        off = 4096;
    }
    else if (b >= 4095) {
        on = 4096;
        off = 0;
    }
    else {
        on = 0;
        off = b;
    }
    i2c_send((on & 0xFF));
    i2c_send(((on >> 8) & 0x1F));
    i2c_send((off & 0xFF));
    i2c_send(((off >> 8) & 0x1F));
    if (c == 0){
        on = 0;
        off = 4096;
    }
    else if (c >= 4095) {
        on = 4096;
        off = 0;
    }
    else {
        on = 0;
        off = c;
    }
    i2c_send((on & 0xFF));
    i2c_send(((on >> 8) & 0x1F));
    i2c_send((off & 0xFF));
    i2c_send(((off >> 8) & 0x1F));
    if (d == 0){
        on = 0;
        off = 4096;
    }
    else if (d >= 4095) {
        on = 4096;
        off = 0;
    }
    else {
        on = 0;
        off = d;
    }
    i2c_send((on & 0xFF));
    i2c_send(((on >> 8) & 0x1F));
    i2c_send((off & 0xFF));
    i2c_send(((off >> 8) & 0x1F));
    i2c_stop();
}

#endif