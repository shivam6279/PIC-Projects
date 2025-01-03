#include "BLDC.h"
#include "PWM.h"
#include <xc.h>
#include <stdbool.h>
#include <inttypes.h>
#include <sys/attribs.h>
#include <math.h>
#include "USART.h"
#include "pic32.h"

unsigned char pole_pairs = POLE_PAIRS;

const float svpwm_max = 1000;
const unsigned int SVPWM_table[SVPWM_SIZE] = {500, 515, 530, 545, 560, 575, 591, 606, 621, 635, 650, 665, 680, 695, 710, 724, 739, 753, 768, 782, 796, 810, 824, 838, 852, 866, 880, 893, 907, 920, 933, 937, 941, 946, 949, 953, 957, 960, 964, 967, 970, 973, 976, 978, 981, 983, 985, 987, 989, 991, 992, 994, 995, 996, 997, 998, 999, 999, 1000, 1000, 1000, 1000, 1000, 999, 999, 998, 997, 996, 995, 994, 992, 991, 989, 987, 985, 983, 981, 978, 976, 973, 970, 967, 964, 960, 957, 953, 949, 946, 941, 937, 933, 937, 941, 946, 949, 953, 957, 960, 964, 967, 970, 973, 976, 978, 981, 983, 985, 987, 989, 991, 992, 994, 995, 996, 997, 998, 999, 999, 1000, 1000, 1000, 1000, 1000, 999, 999, 998, 997, 996, 995, 994, 992, 991, 989, 987, 985, 983, 981, 978, 976, 973, 970, 967, 964, 960, 957, 953, 949, 946, 941, 937, 933, 920, 907, 893, 880, 866, 852, 838, 824, 810, 796, 782, 768, 753, 739, 724, 710, 695, 680, 665, 650, 635, 621, 606, 591, 575, 560, 545, 530, 515, 500, 485, 470, 455, 440, 425, 409, 394, 379, 365, 350, 335, 320, 305, 290, 276, 261, 247, 232, 218, 204, 190, 176, 162, 148, 134, 120, 107, 93, 80, 67, 63, 59, 54, 51, 47, 43, 40, 36, 33, 30, 27, 24, 22, 19, 17, 15, 13, 11, 9, 8, 6, 5, 4, 3, 2, 1, 1, 0, 0, 0, 0, 0, 1, 1, 2, 3, 4, 5, 6, 8, 9, 11, 13, 15, 17, 19, 22, 24, 27, 30, 33, 36, 40, 43, 47, 51, 54, 59, 63, 67, 63, 59, 54, 51, 47, 43, 40, 36, 33, 30, 27, 24, 22, 19, 17, 15, 13, 11, 9, 8, 6, 5, 4, 3, 2, 1, 1, 0, 0, 0, 0, 0, 1, 1, 2, 3, 4, 5, 6, 8, 9, 11, 13, 15, 17, 19, 22, 24, 27, 30, 33, 36, 40, 43, 47, 51, 54, 59, 63, 67, 80, 93, 107, 120, 134, 148, 162, 176, 190, 204, 218, 232, 247, 261, 276, 290, 305, 320, 335, 350, 365, 379, 394, 409, 425, 440, 455, 470, 485};

double encoder_calib_data[] = {7.6464, 19.1601, 30.2343, 40.6054, 52.0312, 62.7539, 73.9160, 85.3417, 96.2402, 107.4023, 119.6191, 130.8691, 142.4707, 154.3359, 165.6738, 176.9238, 187.9101, 199.0722, 210.0585, 221.0449, 231.5039, 242.2265, 253.7402, 264.4628, 275.2734, 287.4902, 298.7402, 310.2539, 322.3828, 333.8085, 345.3222, 356.5722};
double encoder_LUT[(int)ENCODER_RES];

volatile unsigned char mode = MODE_POWER, waveform_mode = WAVEFORM_FOC;

volatile float pre_pos = 0, position = 0.0, rpm = 0.0, rpm_der = 0.0, power = 0.0;
volatile float set_rpm = 0, set_pos = 0.0;
volatile uint8_t current_six_step_phase = 0;
volatile bool vel_hold = false;

float kp = 8.0;
float ki = 1.25;
float kd = 0.2;
float pre_err = 0.0, err, sum = 0;

volatile float motor_zero_angle = 0.0;
volatile float motor_zero_step_offset = 4.76;
volatile float six_step_angle;

#define INTEGRAL_MAX 150

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL6AUTO) FOC_loop(void){
	IFS0bits.T4IF = 0;
	
	static long int pos_cnt, ind_cnt;
	pos_cnt = POS1CNT;
	ind_cnt = INDX1CNT;
	
	if(pos_cnt < 0) {
		pos_cnt += ENCODER_RES;
	}
	pos_cnt &= ENCODER_RES_MASK;
	
//	position = encoder_LUT[pos_cnt] - motor_zero_angle;
		position = ((float)pos_cnt * 360.0 / ENCODER_RES) - motor_zero_angle;
	
	while(position < 0.0) {
		position += 360.0;
	}
	while(position > 360.0) {
		position -= 360.0;
	}
		
		six_step_angle = fmod(position, 30) / 6;
	current_six_step_phase = ((signed int)six_step_angle + 6) % 6;
	
	if(mode == MODE_POS) {
		position += ind_cnt * 360.0;
		
		if((position - pre_pos) > 60) {
			position -= 360.0;
		} else if((position - pre_pos) < -60) {
			position += 360.0;
		}
		pre_pos = position;
		
		err = set_pos - position;
		sum += err / 50000.0;
		sum = sum >= INTEGRAL_MAX ? INTEGRAL_MAX: sum <= -INTEGRAL_MAX ? -INTEGRAL_MAX: sum;
		
		if((pre_err < 0 && err > 0) || (pre_err > 0 && err < 0))
			sum = 0;
		pre_err = err;
		
		power = kp*err + ki*sum;
		if((rpm > 4 || rpm < -4) && (err < 5 || err > -5)) {
			power -= kd*rpm;
		}
		power = power >= 800 ? 0.4: power <= -800 ? -0.4: power/2000.0;
	}
	
	if(mode != MODE_OFF) {
		if(waveform_mode == WAVEFORM_FOC) {
			setPhaseVoltage(power, position * POLE_PAIRS + 90);
		} else if(waveform_mode == WAVEFORM_TRAPEZOID) {
			MotorPhase(current_six_step_phase, power);
		}
	}
}

volatile float rpm_err, rpm_out, rpm_sum, p_rpm = 0.0;
#define RPM_LPF 0.95
#define RPM_DER_LPF 0.9

const float rpm_kp = 0.003, rpm_ki = 0.01, rpm_kd = 0.002;
const float rpm_maxp = 1.0;

void __ISR_AT_VECTOR(_TIMER_6_VECTOR, IPL4AUTO) RPM(void){
	IFS2bits.T6IF = 0;
	static long int temp = 0, p_temp;
	static int i;
	
	p_temp = temp;
	temp = VEL1CNT;
	
	p_rpm = rpm;
	rpm = (1.0-RPM_LPF) * ((float)temp / ENCODER_RES * 30000.0) + RPM_LPF*rpm;
	
	//rpm_der = (1.0-RPM_LPF) * ((float)(temp-p_temp) / ENCODER_RES * 60000.0) + RPM_LPF*rpm_der;    
	rpm_der = (1.0-RPM_DER_LPF)*100.0*(rpm-p_rpm) + RPM_DER_LPF*rpm_der;
	
	if(mode == MODE_RPM) {        
		rpm_err = set_rpm - rpm;
		if(fabs(rpm_err/set_rpm) < 0.1) {
			rpm_sum += rpm_err * 0.002;
		} else {
			rpm_sum = 0.0;
		}
		//rpm_sum = rpm_sum > 75 ? 75: rpm_sum < -75 ? -75: rpm_sum;
		rpm_out = rpm_kp * rpm_err + rpm_ki * rpm_sum;// - rpm_kd * rpm_der;
		
		if(set_rpm == 0 && fabs(rpm) < 25) {
			power = 0;
		} else {
			power = rpm_out > rpm_maxp ? rpm_maxp: rpm_out < -rpm_maxp ? -rpm_maxp: rpm_out;
		}
	}
}

inline void setPhaseVoltage(float p, float angle_el) {
	float pwm_a, pwm_b, pwm_c;
	
	p = p < -1.0 ? -1.0 : p > 1.0 ? 1.0 : p;
	
	#if FOC_MODE == 0
		//FOCModulationType::SinePWM :

		angle_el = normalizeAngle(angle_el + zero_electric_angle);
		Ualpha =  -sinTable[floor(angle_el * ANGLE_STEP)] * Uq;
		Ubeta =  cosTable[floor(angle_el * ANGLE_STEP)] * Uq;
		Ua = Ualpha + voltage_power_supply/2;
		Ub = -0.5 * Ualpha  + SQRT3_BY2 * Ubeta + voltage_power_supply/2;
		Uc = -0.5 * Ualpha - SQRT3_BY2 * Ubeta + voltage_power_supply/2;

	#elif FOC_MODE == 1
		// FOCModulationType::SpaceVectorPWM :
		int index;

		if(p < 0) angle_el += 180;
		p = fabs(p);
		
		p *= (float)PWM_MAX;

		// angle normalisation in between 0 and 2pi
		angle_el = normalizeAngle(angle_el);
		
		index = angle_el;
		index = index < 0.0 | index >= SVPWM_SIZE ? 0 : index;
		
		pwm_a = (float)SVPWM_table[index]/svpwm_max * p;
		
		index = (index + SVPWM_INCREMENT) % SVPWM_SIZE;
		pwm_b = (float)SVPWM_table[index]/svpwm_max * p;
		
		index = (index + SVPWM_INCREMENT) % SVPWM_SIZE;
		pwm_c = (float)SVPWM_table[index]/svpwm_max * p;
		
		PDC1 = pwm_a;
		PDC7 = pwm_a;

		PDC2 = pwm_b;
		PDC8 = pwm_b;

		PDC3 = pwm_c;
		PDC9 = pwm_c;
		
	#elif FOC_MODE == 2
		// Standard phase commutation
		if(p < 0) angle_el += 180;
		p = fabs(p);
		angle_el = floor(normalizeAngle(angle_el + ZERO_ANGLE + 90) / 60) + 1;
		signed char s = angle_el;
		
		MotorPhase(s, p * PWM_MAX);
	#endif
}

void ResetMotorPID() {
	err = 0.0;
	pre_err = 0.0;
	sum = 0.0;
	set_pos = position;
	
	power = 0.0;
	
	rpm_err = 0.0;
	rpm_sum = 0.0;
}

void SetPower(float p) {
	power = p;
}

void SetRPM(float rpm) {
	set_rpm = rpm;
}

void SetPosition(float pos) {
	set_pos = pos;
}

float GetPosition() {
	return position;
}

float GetPower() {
	return power;
}

void ResetPosition() {
	unsigned long int t = VEL1CNT;
	INDX1CNT = 0;
	position = 0.0;
}

float GetRPM() {
	return rpm;
}

float GetRPM_der() {
	return rpm_der;
}

float normalizeAngle(float angle) {
	float a = fmod(angle, 360);
	return a >= 0 ? a : (a + 360);
}

void MotorPhase(signed char num, float val) {
	val = val * (float)PWM_MAX;
	num = num % 6;
	if(num < 0) {
		num += 6;
	}
	switch(num) {
		case 0:
			// W - NC
			PDC1 = 0;
			PDC7 = PWM_MAX;
			// V - V+
			PDC2 = val;
			PDC8 = val;
			// U - GND
			PDC3 = 0;
			PDC9 = 0;
			break;
		case 1:
			// W - V+
			PDC1 = val;
			PDC7 = val;
			// V - NC
			PDC2 = 0;
			PDC8 = PWM_MAX;
			// U - GND
			PDC3 = 0;
			PDC9 = 0;
			break;
		case 2:
			// W - V+
			PDC1 = val;
			PDC7 = val;
			// V - GND
			PDC2 = 0;
			PDC8 = 0;
			// U - NC
			PDC3 = 0;
			PDC9 = PWM_MAX;
			break;
		case 3:
			// W - NC
			PDC1 = 0;
			PDC7 = PWM_MAX;
			// V - GND
			PDC2 = 0;
			PDC8 = 0;
			// U - V+
			PDC3 = val;
			PDC9 = val;
			break;
		case 4:
			// W - GND
			PDC1 = 0;
			PDC7 = 0;
			// V - NC
			PDC2 = 0;
			PDC8 = PWM_MAX;
			// U - V+
			PDC3 = val;
			PDC9 = val;
			break;
		case 5:
			// W - GND
			PDC1 = 0;
			PDC7 = 0;
			// V - V+
			PDC2 = val;
			PDC8 = val;
			// U - NC
			PDC3 = 0;
			PDC9 = PWM_MAX;
			break;       
	}
}

void MotorOff() {
	power = 0;
	// A - NC
	PDC1 = 0;
	PDC7 = PWM_MAX;
	// B - NC
	PDC2 = 0;
	PDC8 = PWM_MAX;
	// C - NC
	PDC3 = 0;
	PDC9 = PWM_MAX;
}

void init_encoder_lut() {
	unsigned int i;
	for(i = 0; i < ENCODER_RES; i++) {
		encoder_LUT[i] = (double)i * 360.0/(double)ENCODER_RES;
	}
}

void interpolate_encoder_lut(double in[], unsigned int len) {
	int i, j, k;
	double delta;
	double arr[(int)POLE_PAIRS*6][2];
	
	for(i = 0; i < len; i++) {
		arr[i][0] = in[i] * ENCODER_RES / 360.0;
		arr[i][1] = (double)i / len * ENCODER_RES;
	}
	
	for(i = 0; i < ENCODER_RES; i++) {
		for(j = 0; j < len; j++) {
			k = (j + 1) % len;
			if(i >= arr[j][0] && i < arr[k][0]) {
				delta = i - arr[j][0];
				delta = delta / (arr[k][0] - arr[j][0]) * ENCODER_RES / len;//(arr[k][1] - arr[j][1]);
				break;
			} else if((fabs(arr[k][0] - arr[j][0]) > ENCODER_RES/2) && (i > arr[j][0] || i < arr[k][0])) {
				delta = i - arr[j][0];
				if(delta > ENCODER_RES/2) {
					delta -= ENCODER_RES;
				} else if(delta < -ENCODER_RES/2) {
					delta += ENCODER_RES;
				}
				delta = delta / (arr[k][0] - arr[j][0] + ENCODER_RES) * ENCODER_RES / len;// * (arr[k][1] - arr[j][1]);
				break;
			}
		}
		encoder_LUT[i] = fmod(arr[j][1] + delta, ENCODER_RES) * 360.0 / ENCODER_RES;
	}
}
