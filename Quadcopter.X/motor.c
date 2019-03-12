#include "motor.h"
#include "PWMDriver.h"
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
    write_pwm(MOTOR_UPRIGHT_PIN, MOTOR_MAX);
    write_pwm(MOTOR_DOWNLEFT_PIN, MOTOR_MAX);
    write_pwm(MOTOR_UPLEFT_PIN, MOTOR_MAX);
    write_pwm(MOTOR_DOWNRIGHT_PIN, MOTOR_MAX);
    delay_ms(4500);
}

void WriteMotors(Motors x) {
    float ur = (float)x.upRight / MAX_SPEED * (MOTOR_MAX - MOTOR_OFF);
    float dl = (float)x.downLeft / MAX_SPEED * (MOTOR_MAX - MOTOR_OFF);
    float ul = (float)x.upLeft / MAX_SPEED * (MOTOR_MAX - MOTOR_OFF);
    float dr = (float)x.downRight / MAX_SPEED * (MOTOR_MAX - MOTOR_OFF);
    write_pwm(MOTOR_UPRIGHT_PIN, MOTOR_OFF + (int)ur);
    write_pwm(MOTOR_DOWNLEFT_PIN, MOTOR_OFF + (int)dl);
    write_pwm(MOTOR_UPLEFT_PIN, MOTOR_OFF + (int)ul);
    write_pwm(MOTOR_DOWNRIGHT_PIN, MOTOR_OFF + (int)dr); 
}

void TurnMotorsOff() {
    write_pwm(MOTOR_UPRIGHT_PIN, MOTOR_OFF);
    write_pwm(MOTOR_DOWNLEFT_PIN, MOTOR_OFF);
    write_pwm(MOTOR_UPLEFT_PIN, MOTOR_OFF);
    write_pwm(MOTOR_DOWNRIGHT_PIN, MOTOR_OFF);
}
