#include "motor.h"
#include "PWMDriver.h"
#include "settings.h"

void LimitSpeed(Motors *x){
    if(x->upRight < 0) x->upRight = 0;
    else if(x->upRight > (MOTOR_MAX - MOTOR_OFF)) x->upRight = (MOTOR_MAX - MOTOR_OFF);
    if(x->downRight < 0) x->downRight = 0;
    else if(x->downRight > (MOTOR_MAX - MOTOR_OFF)) x->downRight = (MOTOR_MAX - MOTOR_OFF);
    if(x->upLeft < 0) x->upLeft = 0;
    else if(x->upLeft > (MOTOR_MAX - MOTOR_OFF)) x->upLeft = (MOTOR_MAX - MOTOR_OFF);
    if(x->downLeft < 0) x->downLeft = 0;
    else if(x->downLeft > (MOTOR_MAX - MOTOR_OFF)) x->downLeft = (MOTOR_MAX - MOTOR_OFF);
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
    write_pwm(MOTOR_UPRIGHT_PIN, MOTOR_OFF + x.upRight);
    write_pwm(MOTOR_DOWNLEFT_PIN, MOTOR_OFF + x.downLeft);
    write_pwm(MOTOR_UPLEFT_PIN, MOTOR_OFF + x.upLeft);
    write_pwm(MOTOR_DOWNRIGHT_PIN, MOTOR_OFF + x.downRight); 
}

void TurnMotorsOff() {
    write_pwm(MOTOR_UPRIGHT_PIN, MOTOR_OFF);
    write_pwm(MOTOR_DOWNLEFT_PIN, MOTOR_OFF);
    write_pwm(MOTOR_UPLEFT_PIN, MOTOR_OFF);
    write_pwm(MOTOR_DOWNRIGHT_PIN, MOTOR_OFF);
}
