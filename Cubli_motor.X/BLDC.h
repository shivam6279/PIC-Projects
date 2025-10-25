#ifndef _BLDC_H
#define _BLDC_H

#include <math.h>
#include <inttypes.h>
#include <stdbool.h>
#include <xc.h>
#include "PID.h"

#define ENCODER_RES 4096.0f
#define ENCODER_RES_MASK 0xFFF

#define FOC_DEGREE_ADVANCE 90.0f
#define RPM_ADVANCE_FACTOR 0.0002f

#define LUT_SIZE 720 // For one quadrant of a time period
#define LUT_120_SHIFT (LUT_SIZE / 3)

#define FOC_TIMER_ON T4CONbits.ON

typedef enum {
	MOTOR_SVPWM,
	MOTOR_SIN,
	MOTOR_FOC,
	MOTOR_SADDLE,
	MOTOR_TRAPEZOID,
	MODE_SENSORLESS
} motor_waveform_type;

typedef enum {
	MODE_OFF,
	MODE_POWER,
	MODE_RPM,
	MODE_POS
} motor_mode;

extern volatile motor_waveform_type waveform_mode;
extern volatile motor_mode mode;

extern PID pid_angle, pid_rpm, pid_focIq, pid_focId;

extern volatile float phase_delay;
#define LPF_PHASE 0.5f

extern float motor_zero_angle;
extern double encoder_LUT[];
extern double encoder_calib_data[32];

extern volatile unsigned char comparator, comp_u, comp_v, comp_w;

extern float motor_pole_pairs, foc_degree_advance;

extern volatile float foc_id, foc_iq;
extern volatile float isns_u, isns_v, isns_w;

extern void init_encoder_lut();
extern void interpolate_encoder_lut(double[], unsigned int);

extern void MotorPIDInit();
extern void MotorPhase(signed char, float);
extern void MotorPhasePWM(float, float, float);
extern void MotorOff();
extern void MotorShort(float);

extern bool bemf_phase(unsigned char);

extern void SensorlessStart(float);

extern void setPhaseVoltage(float, float, float);
extern void foc_current_calc(float);
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
extern float wave_lut(uint16_t*, float);

#endif
