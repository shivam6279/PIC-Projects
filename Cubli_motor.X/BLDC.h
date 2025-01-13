#ifndef _BLDC_H
#define _BLDC_H

#include <math.h>
#include <inttypes.h>

#define ENCODER_RES 4096.0f
#define ENCODER_RES_MASK 0xFFF

#define FOC_DEGREE_ADVANCE 90.0f
#define RPM_ADVANCE_FACTOR 0.000f//0.0002f

#define SVPWM_SIZE 1440
#define SVPWM_INCREMENT (SVPWM_SIZE / 3)

#define FOC_TIMER_ON T4CONbits.ON

typedef enum {
	MOTOR_SVPWM,
	MOTOR_SIN,
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

extern volatile float phase_delay;
#define LPF_PHASE 0.1f

extern volatile float motor_zero_angle;
extern double encoder_LUT[];
extern double encoder_calib_data[32];

extern float pole_pairs;

extern void init_encoder_lut();
extern void interpolate_encoder_lut(double[], unsigned int);

extern void MotorPhase(signed char, float);
extern void MotorOff();
extern unsigned char GetHallPhase();
extern unsigned char TestPhase(int, int);

extern void SensorlessStart(float);

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
