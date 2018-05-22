#ifndef _PID_H_
#define _PID_H_

typedef struct{
    float p, i, d;
    float error, sum;
    float offset;
    float output;
} PID;

extern void LimitAngle(float*);
extern void WriteRGBLed(unsigned int, unsigned int, unsigned int);
extern void PIDSet(PID*, float, float, float);
extern void StrWriteInt(int, unsigned char, char[], unsigned char);
extern void StrWriteFloat(double, unsigned char, unsigned char, char[], unsigned char);
extern void WriteRGBLed(unsigned int, unsigned int, unsigned int);

int loop_counter = 0;

#endif