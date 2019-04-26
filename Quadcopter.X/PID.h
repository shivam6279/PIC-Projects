#ifndef _PID_H_
#define _PID_H_

typedef struct {
    float p, i, d;
    float error, p_error; 
    float sum, derivative;
    float offset;
    float output;
} PID;

extern volatile unsigned long int esc_counter;
extern volatile unsigned long int data_aq_counter;
extern volatile unsigned char altitude_timer;
extern volatile unsigned int ToF_counter;

extern void SetPIDGain(PID*, PID*, PID*, PID*, PID*);
extern float LimitAngle(float);
extern float LimitValue(float, float, float)
extern void WriteRGBLed(unsigned int, unsigned int, unsigned int);
extern void PIDSet(PID*, float, float, float);
extern void PIDIntegrate(PID*, float);
extern void PIDIntegrateAngle(PID*, float);
extern void PIDDifferentiate(PID*, float);
extern void PIDCompute(PID*);

extern void StrWriteInt(int, volatile char[], unsigned char);
extern void StrWriteFloat(double, unsigned char, volatile char[], unsigned char);
extern void WriteRGBLed(unsigned int, unsigned int, unsigned int);

extern float max_pitch_roll_tilt, max_yaw_rate;

#endif