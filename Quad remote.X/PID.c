#include "PID.h"

void LimitAngle(float *a){
    while(*a < (-180) || *a > 180){ if(*a < (-180)) *a += 360; else if(*a > 180) *a -= 360; }
}

void LimitSpeed(Motors *x){
    if(x->up < 0) x->up = 0;
    else if(x->up > (MOTOR_MAX - MOTOR_OFF)) x->up = (MOTOR_MAX - MOTOR_OFF);
    if(x->down < 0) x->down = 0;
    else if(x->down > (MOTOR_MAX - MOTOR_OFF)) x->down = (MOTOR_MAX - MOTOR_OFF);
    if(x->right < 0) x->right = 0;
    else if(x->right > (MOTOR_MAX - MOTOR_OFF)) x->right = (MOTOR_MAX - MOTOR_OFF);
    if(x->left < 0) x->left = 0;
    else if(x->left > (MOTOR_MAX - MOTOR_OFF)) x->left = (MOTOR_MAX - MOTOR_OFF);
}

void PIDReset(PID *x, float kp, float ki, float kd){
    x->p = kp; 
    x->i = ki; 
    x->d = kd; 
    x->sum = 0.0f; 
    x->output = 0.0f; 
    x->error = 0.0f;
}

void MotorsReset(Motors *x){
    x->up = 0;
    x->down = 0; 
    x->left = 0; 
    x->right = 0;
}

void StrWriteInt(int a, unsigned char precision, char str[], unsigned char n){
    if(a < 0){ a *= (-1); str[n++] = '-'; }
    else str[n++] = '+';
    if(precision >= 6) str[n++] = ((a / 100000) % 10) + 48;
    if(precision >= 5) str[n++] = ((a / 10000) % 10) + 48;
    if(precision >= 4) str[n++] = ((a / 1000) % 10) + 48;
    if(precision >= 3) str[n++] = ((a / 100) % 10) + 48;
    if(precision >= 2) str[n++] = ((a / 10) % 10) + 48;
    if(precision >= 1) str[n++] = (a % 10) + 48;
}

void StrWriteFloat(double a, unsigned char left, unsigned char right, char str[], unsigned char n){
    unsigned char i;
    long int tens = 10;
    if(a < 0){ a *= (-1); str[n++] = '-'; }
    else str[n++] = '+';
    if(left >= 7) str[n++] = ((int)(a / 1000000) % 10) + 48;
    if(left >= 6) str[n++] = ((int)(a / 100000) % 10) + 48;
    if(left >= 5) str[n++] = ((int)(a / 10000) % 10) + 48;
    if(left >= 4) str[n++] = ((int)(a / 1000) % 10) + 48;
    if(left >= 3) str[n++] = ((int)(a / 100) % 10) + 48;
    if(left >= 2) str[n++] = ((int)(a / 10) % 10) + 48;
    if(left >= 1) str[n++] = ((int)a % 10) + 48;
    str[n++] = '.';
    for(i = 0; i < right; i++, tens *= 10, n++) str[n] = ((long int)(a * tens) % 10) + 48;
}

void WriteRGBLed(unsigned int r, unsigned int g, unsigned int b){
    write_pwm(RGBled_red_pin, r); write_pwm(RGBled_green_pin, g); write_pwm(RGBled_blue_pin, b);
}
