#include "PID.h"
#include "pic32.h"
#include <math.h>
#include "XBee.h"
#include <sys/attribs.h>

volatile unsigned long int esc_counter = 0;
volatile unsigned long int gyro_aq_counter = 0, acc_aq_counter = 0, compass_aq_counter = 0;
volatile unsigned char altitude_timer = 0;
volatile unsigned int ToF_counter = 0;
volatile unsigned int tx_buffer_timer = 0;

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL7AUTO) pid_loop_timer(void){
    IFS0bits.T4IF = 0;
    esc_counter++;
    gyro_aq_counter++;
    acc_aq_counter++;
    compass_aq_counter++;
}

void __ISR_AT_VECTOR(_TIMER_7_VECTOR, IPL4AUTO) general_purpose_1KHz(void) {
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
    
    gyro_aq_counter = 0;
    acc_aq_counter = 0;
    compass_aq_counter = 0;

    altitude_timer = 0;
    ToF_counter = 0;
    tx_buffer_timer = 0;
}


void SetPIDGain(PID *roll, PID* pitch, PID *yaw) {
    PIDSet(roll,     1.30, 1.30, 0.50);
    PIDSet(pitch,    1.30, 1.30, 0.50);
    PIDSet(yaw,      1.30, 1.30, 0.50);
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
    if(a < min) {
        return min;
    } else if(a > max) {
        return max;
    }
    return a;
}

void PIDSet(PID *x, float p, float i, float d) {
    x->kp = p; 
    x->ki = i; 
    x->kd = d; 
    x->integral = 0.0f; 
    x->derivative = 0.0f;
    x->output = 0.0f; 
    x->error = 0.0f;
    x->p_error = 0.0f;
    x->offset = 0.0f;
    
    x->integral_bound = 0.0f;
    x->integral_max_diff = 0.0f;
}

void PIDSetIntegralParams(PID *x, float int_bound, float int_diff) {
    x->integral_bound = int_bound;
    x->integral_max_diff = int_diff;
}

void PIDReset(PID* x) {
    x->integral = 0.0f; 
    x->derivative = 0.0f;
    x->output = 0.0f; 
    x->error = 0.0f;
    x->p_error = 0.0f;
    x->offset = 0.0f;
}

void PIDIntegrate(PID *a, float deltat) {
    if(a->integral_max_diff <= 0 || fabs(a->error - a->offset) <= a->integral_max_diff) {   
        a->integral += (a->error - a->offset) * deltat;
    }
    
    if(a->integral_bound > 0.0f) {
        if(a->integral * a->ki > a->integral_bound) {
            a->integral = a->integral_bound / a->ki;
        } else if(a->integral * a->ki < -a->integral_bound) {
            a->integral = -a->integral_bound / a->ki;
        }
    }
}

void PIDIntegrateAngle(PID *a, float deltat) {    
    if(a->integral_max_diff <= 0 || fabs(LimitAngle(a->error - a->offset)) <= a->integral_max_diff) {   
        a->integral += LimitAngle(a->error - a->offset) * deltat;
    }
    
    if(a->integral_bound > 0.0f) {
        if(a->integral * a->ki > a->integral_bound) {
            a->integral = a->integral_bound / a->ki;
        } else if(a->integral * a->ki < -a->integral_bound) {
            a->integral = -a->integral_bound / a->ki;
        }
    }
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
    a->output = a->kp * (a->error - a->offset) + a->ki * a->integral + a->kd * a->derivative;
}

void PIDOutputAngle(PID *a) {
    a->output = a->kp * LimitAngle(a->error - a->offset) + a->ki * a->integral + a->kd * a->derivative;
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