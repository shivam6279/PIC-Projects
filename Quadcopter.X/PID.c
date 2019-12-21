#include "PID.h"
#include "settings.h"
#include "PWM.h"
#include "pic32.h"
#include <math.h>
#include "XBee.h"
#include <sys/attribs.h>

volatile unsigned long int esc_counter = 0;
volatile unsigned long int data_aq_counter = 0;
volatile unsigned char altitude_timer = 0;
volatile unsigned int ToF_counter = 0;
volatile unsigned int tx_buffer_timer = 0;

float max_pitch_roll_tilt = MAX_PITCH_ROLL_TILT;
float max_yaw_rate = MAX_YAW_RATE;

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL7SRS) pid_loop_timer(void){
    IFS0bits.T4IF = 0;
    esc_counter++;
    data_aq_counter++;
}

void __ISR_AT_VECTOR(_TIMER_7_VECTOR, IPL4SRS) general_purpose_1KHz(void) {
    IFS1bits.T7IF = 0;
    altitude_timer++;
    ToF_counter++;
    tx_buffer_timer++;
    if(safety_counter < 500) {
        safety_counter++;
    } 
    else if(safety_counter == 500) {
        XBeeReset();
        safety_counter = 501;
    }
}

void ResetCounters() {
    esc_counter = 0;
    data_aq_counter = 0;

    altitude_timer = 0;
    ToF_counter = 0;
    tx_buffer_timer = 0;
}


void SetPIDGain(PID *roll, PID* pitch, PID *yaw, PID *altitude, PID *GPS) {
#ifdef micro
    PIDSet(roll,     1.00, 1.00, 0.30);
    PIDSet(pitch,    1.00, 1.00, 0.30);
    PIDSet(yaw,      0.60, 1.80, 0.50);
    PIDSet(altitude, 36.0, 5.00, 5.00);
    PIDSet(GPS,      1.50, 0.05, 0.00);
#endif
#ifdef mini
    PIDSet(roll,     1.10, 1.10, 0.30);
    PIDSet(pitch,    1.10, 1.10, 0.30);
    PIDSet(yaw,      1.10, 1.10, 0.30);
    PIDSet(altitude, 36.0, 0.80, 0.00);
    PIDSet(GPS,      1.50, 0.05, 0.00);
#endif
#ifdef big
    PIDSet(roll,     0.30, 1.20, 0.40);
    PIDSet(pitch,    0.30, 1.20, 0.40);
    PIDSet(yaw,      0.40, 1.00, 0.30);
    PIDSet(altitude, 36.0, 0.80, 0.00);
    PIDSet(GPS,      1.50, 0.05, 0.00);
#endif
}

float LimitAngle(float a) {
    while(a < (-180.0) || a > 180.0) { 
        if(a < (-180.0)) 
            a += 360.0; 
        else if(a > 180.0) 
            a -= 360.0; 
    }
    return a;
}

float LimitValue(float a, float min, float max) {
    if(a < min)
        return min;
    if(a > max)
        return max;
    return a;
}

void PIDSet(PID *x, float kp, float ki, float kd) {
    x->p = kp; 
    x->i = ki; 
    x->d = kd; 
    x->sum = 0.0f; 
    x->derivative = 0.0f;
    x->output = 0.0f; 
    x->error = 0.0f;
    x->p_error = 0.0f;
    x->offset = 0.0f;
}

void PIDIntegrate(PID *a, float deltat) {
    a->sum += (a->error - a->offset) * deltat;
}

void PIDIntegrateAngle(PID *a, float deltat) {
    a->sum += LimitAngle(a->error - a->offset) * deltat;
}

void PIDDifferentiate(PID *a, float deltat) {
    a->derivative = (a->error - a->p_error) / deltat;
    a->p_error = a->error;
}

void PIDDifferentiateAngle(PID *a, float deltat) {
    a->derivative = LimitAngle(a->error - a->p_error) / deltat;
    a->p_error = a->error;
}

void PIDOutput(PID *a) {
    a->output = a->p * (a->error - a->offset) + a->i * a->sum + a->d * a->derivative;
}

void PIDOutputAngle(PID *a) {
    a->output = a->p * LimitAngle(a->error - a->offset) + a->i * a->sum + a->d * a->derivative;
}

void StrWriteInt(int a, volatile char str[], unsigned char n) {
    long int tens;
    
    if(a < 0) { 
        a *= (-1); 
        str[n++] = '-'; 
    }
    
    for(tens = 1; tens < a; tens *= 10);
    tens /= 10;

    for(; tens > 0; tens /= 10)
        str[n++] = ((long int)(a / tens) % 10) + 48;
}

void StrWriteFloat(double a, unsigned char right, volatile char str[], unsigned char n) {
    unsigned char i;
    long int tens;
    
    if(a < 0) { 
        a *= (-1.0); 
        str[n++] = '-'; 
    }
    
    if(a > 1.0) {
        for(tens = 1; tens < a; tens *= 10);
        tens /= 10;

        for(; tens > 0; tens /= 10)
            str[n++] = ((long int)(a / tens) % 10) + 48;
    } else {
        str[n++] = '0';
    }

    str[n++] = '.';
    for(i = 0, tens = 10; i < right; i++, tens *= 10, n++)
        str[n] = ((long int)(a * tens) % 10) + 48;
}

void WriteRGBLed(unsigned int r, unsigned int g, unsigned int b) {
#if board_version == 4
    r = (float)r * (float)PWM_MAX / 4095.0f;
    g = (float)g * (float)PWM_MAX / 4095.0f;
    b = (float)b * (float)PWM_MAX / 4095.0f;
#endif
    write_pwm(RGBLED_RED_PIN, r); 
    write_pwm(RGBLED_GREEN_PIN, g); 
    write_pwm(RGBLED_BLUE_PIN, b);
}