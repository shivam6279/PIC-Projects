#ifndef _BLDC_H
#define _BLDC_H

#include <math.h>

#define FOC_MODE 1
#define ENCODER_RES 4096.0f
#define POLE_PAIRS 12.0f //7.0f
#define DEG_PER_POLE_PAIR  (float)(360.0 / POLE_PAIRS)

#define SVPWM_SIZE		360
#define SVPWM_INCREMENT (SVPWM_SIZE / 3)

#define MODE_OFF 0
#define MODE_POWER 1
#define MODE_RPM 2
#define MODE_POS 3

#define WAVEFORM_FOC 0
#define WAVEFORM_TRAPEZOID 1

extern volatile float motor_zero_angle;
extern volatile unsigned char mode, waveform_mode;

extern void MotorPhase(signed char, float);
extern void MotorOff();
extern unsigned char GetHallPhase();
extern unsigned char TestPhase(int, int);

extern void setPhaseVoltage(float, float);
extern float normalizeAngle(float);

extern void ResetMotorPID();

extern void SetRPM(float);
extern void SetPower(float);
extern void SetPosition(float);
extern float GetPosition();
extern float GetPower();
extern void ResetPosition();
extern float GetRPM();
extern float GetRPM_der();

#endif