#include "BLDC.h"
#include "PWM.h"
#include <xc.h>
#include <stdbool.h>
#include <sys/attribs.h>
#include <math.h>
#include "USART.h"
#include "pic32.h"

const unsigned int SVPWM_table[SVPWM_SIZE] = {1250, 1288, 1326, 1363, 1401, 1439, 1476, 1514, 1551, 1589, 1626, 1663, 1700, 1737, 1774, 1810, 1847, 1883, 1919, 1955, 1990, 2026, 2061, 2096, 2131, 2165, 2199, 2233, 2266, 2300, 2333, 2343, 2354, 2364, 2373, 2383, 2392, 2401, 2409, 2417, 2425, 2432, 2439, 2445, 2452, 2457, 2463, 2468, 2473, 2477, 2481, 2485, 2488, 2491, 2493, 2495, 2497, 2498, 2499, 2500, 2500, 2500, 2499, 2498, 2497, 2495, 2493, 2491, 2488, 2485, 2481, 2477, 2473, 2468, 2463, 2457, 2452, 2445, 2439, 2432, 2425, 2417, 2409, 2401, 2392, 2383, 2373, 2364, 2354, 2343, 2333, 2343, 2354, 2364, 2373, 2383, 2392, 2401, 2409, 2417, 2425, 2432, 2439, 2445, 2452, 2457, 2463, 2468, 2473, 2477, 2481, 2485, 2488, 2491, 2493, 2495, 2497, 2498, 2499, 2500, 2500, 2500, 2499, 2498, 2497, 2495, 2493, 2491, 2488, 2485, 2481, 2477, 2473, 2468, 2463, 2457, 2452, 2445, 2439, 2432, 2425, 2417, 2409, 2401, 2392, 2383, 2373, 2364, 2354, 2343, 2333, 2300, 2266, 2233, 2199, 2165, 2131, 2096, 2061, 2026, 1990, 1955, 1919, 1883, 1847, 1810, 1774, 1737, 1700, 1663, 1626, 1589, 1551, 1514, 1476, 1439, 1401, 1363, 1326, 1288, 1250, 1212, 1174, 1137, 1099, 1061, 1024, 986, 949, 911, 874, 837, 800, 763, 726, 690, 653, 617, 581, 545, 510, 474, 439, 404, 369, 335, 301, 267, 234, 200, 167, 157, 146, 136, 127, 117, 108, 99, 91, 83, 75, 68, 61, 55, 48, 43, 37, 32, 27, 23, 19, 15, 12, 9, 7, 5, 3, 2, 1, 0, 0, 0, 1, 2, 3, 5, 7, 9, 12, 15, 19, 23, 27, 32, 37, 43, 48, 55, 61, 68, 75, 83, 91, 99, 108, 117, 127, 136, 146, 157, 167, 157, 146, 136, 127, 117, 108, 99, 91, 83, 75, 68, 61, 55, 48, 43, 37, 32, 27, 23, 19, 15, 12, 9, 7, 5, 3, 2, 1, 0, 0, 0, 1, 2, 3, 5, 7, 9, 12, 15, 19, 23, 27, 32, 37, 43, 48, 55, 61, 68, 75, 83, 91, 99, 108, 117, 127, 136, 146, 157, 167, 200, 234, 267, 301, 335, 369, 404, 439, 474, 510, 545, 581, 617, 653, 690, 726, 763, 800, 837, 874, 911, 949, 986, 1024, 1061, 1099, 1137, 1174, 1212};

volatile unsigned char motor_mode = MODE_OFF;

volatile float pre_pos = 0, position = 0.0, rpm = 0.0, rpm_der = 0.0, power = 0.0;
volatile float set_rpm = 0, set_pos = 0.0;
volatile bool vel_hold = false;

float kp = 3.0;
float ki = 1.0;
float kd = 15.0;
float pre_err = 0.0, err, sum = 0;

volatile float motor_zero_angle = 0.0;

#define INTEGRAL_MAX 150

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL6AUTO) FOC_loop(void){
    IFS0bits.T4IF = 0;
    
    static long int temp, temp2;
    temp = POS1CNT;
    temp2 = INDX1CNT;
    
    position = -temp * 360.0f / ENCODER_RES;

    while(position < 0.0)
        position += 360.0;
    while(position > 360.0)
        position -= 360.0;
    
    if(motor_mode == MODE_POS) {
        position += temp2 * 360.0;
        
        if((position - pre_pos) > 60) {
            position -= 360.0;
        } else if((position - pre_pos) < -60) {
            position += 360.0;
        }
        pre_pos = position;
        
        err = set_pos - position;
        sum += err / 10000.0;
        sum = sum >= INTEGRAL_MAX ? INTEGRAL_MAX: sum <= -INTEGRAL_MAX ? -INTEGRAL_MAX: sum;
        
        if((pre_err < 0 && err > 0) || (pre_err > 0 && err < 0))
            sum = 0;
        pre_err = err;
        
        power = kp*err + ki*sum + kd*-rpm;
        power = power >= 800 ? 0.4: power <= -800 ? -0.4: power/2000.0;
    }
    
    if(motor_mode != MODE_OFF) {
        setPhaseVoltage(power, (position - motor_zero_angle) * POLE_PAIRS + 90);  
    }
}

volatile float rpm_err, rpm_out, rpm_sum, p_rpm = 0.0;
#define RPM_LPF 0.95
#define RPM_DER_LPF 0.9

#define RPM_PWR_LIMIT 0.1 //0.05

void __ISR_AT_VECTOR(_TIMER_6_VECTOR, IPL4AUTO) RPM(void){
    IFS2bits.T6IF = 0;
    static long int temp = 0, p_temp;
    static int i;
    
    p_temp = temp;
    temp = -VEL1CNT;
    
    p_rpm = rpm;
    rpm = (1.0-RPM_LPF) * ((float)temp / ENCODER_RES * 500.0*60.0) + RPM_LPF*rpm;
    
//    rpm_der = (1.0-RPM_LPF) * ((float)(temp-p_temp) / ENCODER_RES * 60000.0) + RPM_LPF*rpm_der;    
    rpm_der = (1.0-RPM_DER_LPF)*100.0*(rpm-p_rpm) + RPM_DER_LPF*rpm_der;
    
    if(motor_mode == MODE_RPM) {        
        
        rpm_err = set_rpm - rpm;
        if(fabs(rpm_err/set_rpm) < 0.5) {
            rpm_sum += rpm_err / 500.0;
        } else {
            rpm_sum = 0.0;
        }        
//        rpm_sum = rpm_sum > 75 ? 75: rpm_sum < -75 ? -75: rpm_sum;
        rpm_out = 0.4*rpm_err + 1.0*rpm_sum + 0.03*rpm_der;
        rpm_out /= 1000;
        
        if(set_rpm == 0 && fabs(rpm) < 25) {
            power = 0;
        } else {
            power = rpm_out > RPM_PWR_LIMIT ? RPM_PWR_LIMIT: rpm_out < -RPM_PWR_LIMIT ? -RPM_PWR_LIMIT: rpm_out;
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

        // angle normalisation in between 0 and 2pi
        angle_el = normalizeAngle(angle_el);
        
        index = angle_el;        
        index = index < 0.0 | index >= SVPWM_SIZE ? 0 : index;
        
        pwm_a = 2.0 * (float)SVPWM_table[index] * p;
        
        index = (index + SVPWM_INCREMENT) % SVPWM_SIZE;
        pwm_b = 2.0 * (float)SVPWM_table[index] * p;
        
        index = (index + SVPWM_INCREMENT) % SVPWM_SIZE;
        pwm_c = 2.0 * (float)SVPWM_table[index] * p;
        
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

inline void MotorPhase(signed char num, float val) {
    val = val * PWM_MAX;
    num = (num - 1) % 6;
    num = num < 0 ? num + 7: num + 1;
    switch(num) {
        case 1:
            // A - GND
            PDC1 = 0;
            PDC7 = 0;
            // B - V+
            PDC2 = val;
            PDC8 = val;
            // C - NC
            PDC3 = PWM_MAX;
            PDC9 = 0;
            break;
        case 2:
            // A - GND
            PDC1 = 0;
            PDC7 = 0;
            // B - NC
            PDC2 = PWM_MAX;
            PDC8 = 0;
            // C - V+
            PDC3 = val;
            PDC9 = val;
            break;
        case 3:
            // A - NC
            PDC1 = PWM_MAX;
            PDC7 = 0;
            // B - GND
            PDC2 = 0;
            PDC8 = 0;
            // C - V+
            PDC3 = val;
            PDC9 = val;
            break;
        case 4:
            // A - V+
            PDC1 = val;
            PDC7 = val;
            // B - GND
            PDC2 = 0;
            PDC8 = 0;
            // C - NC
            PDC3 = PWM_MAX;
            PDC9 = 0;
            break;
        case 5:
            // A - V+
            PDC1 = val;
            PDC7 = val;
            // B - NC
            PDC2 = PWM_MAX;
            PDC8 = 0;
            // C - GND
            PDC3 = 0;
            PDC9 = 0;
            break;
        case 6:
            // A - NC
            PDC1 = PWM_MAX;
            PDC7 = 0;
            // B - V+
            PDC2 = val;
            PDC8 = val;
            // C - GND
            PDC3 = 0;
            PDC9 = 0;
            break;           
    }
}

void MotorOff() {
    power = 0;
    // A - NC
    PDC1 = PWM_MAX;
    PDC7 = 0;
    // B - NC
    PDC2 = PWM_MAX;
    PDC8 = 0;
    // C - NC
    PDC3 = PWM_MAX;
    PDC9 = 0;
}