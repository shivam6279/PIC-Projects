#include "PID.h"
#include "settings.h"
#include "PWMDriver.h"
#include "pic32.h"
#include <math.h>

void SetPIDGain(PID *roll, PID* pitch, PID *yaw, PID *roll_rate, PID *pitch_rate, PID *yaw_rate, PID *altitude, PID *altitude_rate, PID *GPS) {
#ifdef micro
    PIDSet(roll, 1.5, 1.2, 0.0);
    PIDSet(pitch, 1.5, 1.2, 0.0);
    PIDSet(yaw, 1.5, 1.2, 0.0);
    PIDSet(roll_rate, 2.2, 0.0, 0.0);
    PIDSet(pitch_rate, 2.2, 0.0, 0.0);
    PIDSet(yaw_rate, 2.2, 0.0, 0.0);
    PIDSet(altitude, 1.8, 0.04, 0.0);
    PIDSet(altitude_rate, 40.0, 0.0, 0.0);
    PIDSet(GPS, 1.5, 0.05, 0.0);
#endif
#ifdef mini
    PIDSet(roll, 1.5, 1.2, 0.0);
    PIDSet(pitch, 1.5, 1.2, 0.0);
    PIDSet(yaw, 2.0, 1.6, 0.0);
    PIDSet(roll_rate, 2.5, 0.0, 0.0);
    PIDSet(pitch_rate, 2.5, 0.0, 0.0);
    PIDSet(yaw_rate, 3.0, 0.0, 0.0);
    PIDSet(altitude, 1.8, 0.04, 0.0);
    PIDSet(altitude_rate, 40.0, 0.0, 0.0);
    PIDSet(GPS, 1.5, 0.05, 0.0);
#endif
#ifdef big // 1.5, 1.2, 2.7
    PIDSet(roll, 1.2, 1.0, 0.0);
    PIDSet(pitch, 1.2, 1.0, 0.0);
    PIDSet(yaw, 1.2, 1.0, 0.0);
    PIDSet(roll_rate, 0.8, 0.0, 0.0);
    PIDSet(pitch_rate, 0.8, 0.0, 0.0);
    PIDSet(yaw_rate, 1.2, 0.0, 0.0);
    PIDSet(altitude, 1.8, 0.04, 0.0);
    PIDSet(altitude_rate, 40.0, 0.0, 0.0);
    PIDSet(GPS, 1.5, 0.05, 0.0);
#endif
}

void QuaternionToEuler(float q[], PID *roll, PID *pitch, PID *yaw, float *heading, float *yaw_difference, float take_off_heading) {
    roll->p_error = roll->error;
    pitch->p_error = pitch->error;
    yaw->p_error = yaw->error;

    //Converting quaternion to Euler angles
    roll->error = (atan2(2.0f * (q[0] * q[2] - q[3] * q[1]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) + PI) * RAD_TO_DEGREES - ROLLOFFSET;
    pitch->error = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) + PI) * RAD_TO_DEGREES - PITCHOFFSET;
    *heading = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * RAD_TO_DEGREES - HEADINGOFFSET;
    LimitAngle(heading);
    yaw->error = *heading - take_off_heading;  
    LimitAngle(&roll->error);//Limit angles within -180 and +180 degrees
    LimitAngle(&pitch->error);
    LimitAngle(&yaw->error);
    *yaw_difference = yaw->error - yaw->offset;
    LimitAngle(yaw_difference);
}

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
    x->derivative = 0.0f;
    x->output = 0.0f; 
    x->error = 0.0f;
    x->p_error = 0.0f;
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