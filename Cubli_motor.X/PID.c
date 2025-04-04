#include "PID.h"
#include <inttypes.h>

void PID_init(PID *pid) {
	pid->kp = 0;
	pid->ki = 0;
	pid->kd = 0;
	pid->error = 0;
	pid->setpoint = 0;
	pid->p_input = 0;
	pid->integral = 0;
	pid->derivative = 0;
	pid->output = 0;
	pid->integral_max = 0;
	pid->integral_min = 0;
	pid->output_max = 0;
	pid->output_min = 0;

	pid->constrain_integral = false;
	pid->constrain_output = false;
}

void PID_reset(PID *pid) {
	pid->error = 0;
	pid->setpoint = 0;
	pid->p_input = 0;
	pid->integral = 0;
	pid->derivative = 0;
	pid->output = 0;
}

void PID_setGain(PID *pid, float kp, float ki, float kd) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

void PID_integrate(PID *pid, float deltat) {
	pid->integral += pid->error * deltat;
	if(pid->constrain_integral) {
		pid->integral = pid->integral > pid->integral_max ? pid->integral_max: pid->integral < pid->integral_min ? pid->integral_min: pid->integral;
	}
}

void PID_differentiate(PID *pid, float deltat) {
	pid->derivative += (pid->input - pid->p_input) / deltat;
	pid->p_input = pid->input;
}

float PID_compute(PID *pid, float input, float deltat) {
	pid->input = input;
	pid->error = pid->input - pid->setpoint;
	PID_integrate(pid, deltat);
	PID_differentiate(pid, deltat);
	pid->output = pid->kp * pid->error + pid->ki * pid->integral - pid->kd * pid->derivative;
	if(pid->constrain_output) {
		pid->output = pid->output > pid->output_max ? pid->output_max: pid->output < pid->output_min ? pid->output_min: pid->output;
	}
	return pid->output;
}

// Integral constrain
void PID_enableIntegralConstrain(PID *pid) {
	pid->constrain_integral = true;
}

void PID_disableIntegralConstrain(PID *pid) {
	pid->constrain_integral = false;
}

void PID_setIntegralLimits(PID *pid, float min, float max) {
	pid->integral_min = min;
	pid->integral_max = max;
}

// Output constrain
void PID_enableOutputConstrain(PID *pid) {
	pid->constrain_output = true;
}

void PID_disableOutputConstrain(PID *pid) {
	pid->constrain_output = false;
}

void PID_setOutputLimits(PID *pid, float min, float max) {
	pid->output_min = min;
	pid->output_max = max;
}