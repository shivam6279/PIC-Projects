#include "PID.h"
#include "settings.h"
#include "PWMDriver.h"
#include "pic32.h"

void LimitAngle(float *a){
    while(*a < (-180) || *a > 180) { 
        if(*a < (-180)) *a += 360; 
        else if(*a > 180) *a -= 360; 
    }
}

void PIDSet(PID *x, float kp, float ki, float kd) {
    x->p = kp; 
    x->i = ki; 
    x->d = kd; 
    x->sum = 0.0f; 
    x->output = 0.0f; 
    x->error = 0.0f;
    x->offset = 0.0f;
}

void StrWriteInt(int a, unsigned char precision, char str[], unsigned char n) {
    if(a < 0) { 
        a *= (-1); 
        str[n++] = '-'; 
    }
    else str[n++] = '+';
    if(precision >= 6) str[n++] = ((a / 100000) % 10) + 48;
    if(precision >= 5) str[n++] = ((a / 10000) % 10) + 48;
    if(precision >= 4) str[n++] = ((a / 1000) % 10) + 48;
    if(precision >= 3) str[n++] = ((a / 100) % 10) + 48;
    if(precision >= 2) str[n++] = ((a / 10) % 10) + 48;
    if(precision >= 1) str[n++] = (a % 10) + 48;
}

void StrWriteFloat(double a, unsigned char left, unsigned char right, char str[], unsigned char n) {
    unsigned char i;
    long int tens = 10;
    if(a < 0) { 
        a *= (-1.0); 
        str[n++] = '-'; 
    }
    else str[n++] = '+';
    if(left >= 10)str[n++] = ((int)(a / 1000000000.0) % 10) + 48;
    if(left >= 9) str[n++] = ((int)(a / 100000000.0) % 10) + 48;
    if(left >= 8) str[n++] = ((int)(a / 10000000.0) % 10) + 48;
    if(left >= 7) str[n++] = ((int)(a / 1000000.0) % 10) + 48;
    if(left >= 6) str[n++] = ((int)(a / 100000.0) % 10) + 48;
    if(left >= 5) str[n++] = ((int)(a / 10000.0) % 10) + 48;
    if(left >= 4) str[n++] = ((int)(a / 1000.0) % 10) + 48;
    if(left >= 3) str[n++] = ((int)(a / 100.0) % 10) + 48;
    if(left >= 2) str[n++] = ((int)(a / 10.0) % 10) + 48;
    if(left >= 1) str[n++] = ((int)a % 10) + 48;
    str[n++] = '.';
    for(i = 0; i < right; i++, tens *= 10, n++) {
        str[n] = ((long int)(a * tens) % 10) + 48;
    }
}

void WriteRGBLed(unsigned int r, unsigned int g, unsigned int b) {
    write_pwm(RGBLED_RED_PIN, r); 
    write_pwm(RGBLED_GREEN_PIN, g); 
    write_pwm(RGBLED_BLUE_PIN, b);
}