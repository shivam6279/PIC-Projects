#include "motor.h"
#include "PWM.h"
#include "settings.h"

void LimitSpeed(Motors *x){
    if(x->upRight < 0) x->upRight = 0;
    else if(x->upRight > MAX_SPEED) x->upRight = MAX_SPEED;
    if(x->downRight < 0) x->downRight = 0;
    else if(x->downRight > MAX_SPEED) x->downRight = MAX_SPEED;
    if(x->upLeft < 0) x->upLeft = 0;
    else if(x->upLeft > MAX_SPEED) x->upLeft = MAX_SPEED;
    if(x->downLeft < 0) x->downLeft = 0;
    else if(x->downLeft > MAX_SPEED) x->downLeft = MAX_SPEED;
}

void MotorsReset(Motors *x) {
    x->upRight = 0;
    x->downLeft = 0; 
    x->upLeft = 0; 
    x->downRight = 0;
}

void CalibrateESC() {
    TurnMotorsOff();
    delay_ms(500);
    write_pwm(MOTOR_UPRIGHT_PIN, motor_max);
    write_pwm(MOTOR_DOWNLEFT_PIN, motor_max);
    write_pwm(MOTOR_UPLEFT_PIN, motor_max);
    write_pwm(MOTOR_DOWNRIGHT_PIN, motor_max);
    delay_ms(4500);
}

void WriteMotors(Motors x) {
    float ur = (float)x.upRight / MAX_SPEED * (motor_max - motor_off);
    float dl = (float)x.downLeft / MAX_SPEED * (motor_max - motor_off);
    float ul = (float)x.upLeft / MAX_SPEED * (motor_max - motor_off);
    float dr = (float)x.downRight / MAX_SPEED * (motor_max - motor_off);
    write_pwm(MOTOR_UPRIGHT_PIN, motor_off + (int)ur);
    write_pwm(MOTOR_DOWNLEFT_PIN, motor_off + (int)dl);
    write_pwm(MOTOR_UPLEFT_PIN, motor_off + (int)ul);
    write_pwm(MOTOR_DOWNRIGHT_PIN, motor_off + (int)dr); 
}

void TurnMotorsOff() {
    write_pwm(MOTOR_UPRIGHT_PIN, motor_off);
    write_pwm(MOTOR_DOWNLEFT_PIN, motor_off);
    write_pwm(MOTOR_UPLEFT_PIN, motor_off);
    write_pwm(MOTOR_DOWNRIGHT_PIN, motor_off);
}
