#ifndef _PID_H_
#define _PID_H_

#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float error, p_error; 
    float offset;
    float integral, derivative;
    float integral_bound;
    float integral_max_diff;
    float output;
} PID;

extern volatile unsigned long int esc_counter;
extern volatile unsigned long int gyro_aq_counter, acc_aq_counter, compass_aq_counter;
extern volatile unsigned char altitude_timer;
extern volatile unsigned int ToF_counter;
extern volatile unsigned int tx_buffer_timer;

extern void ResetCounters();
extern void SetPIDGain(PID*, PID*, PID*);
extern float LimitAngle(float);
extern float LimitValue(float, float, float);
extern void WriteRGBLed(unsigned int, unsigned int, unsigned int);
extern void PIDSet(PID*, float, float, float);
extern void PIDSetIntegralParams(PID*, float, float);
extern void PIDReset(PID*);
extern void PIDIntegrate(PID*, float);
extern void PIDDifferentiate(PID*, float);
extern void PIDOutput(PID *a);

extern void StrWriteInt(int, volatile char[], unsigned char);
extern void StrWriteFloat(double, unsigned char, volatile char[], unsigned char);

#endif