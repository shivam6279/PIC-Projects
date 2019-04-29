#include "motor.h"
#include "PWM.h"
#include "XBee.h"
#include "PID.h"

void LimitSpeed(Motors *x){
    x->upRight   = LimitValue(x->upRight,   0.0f, MAX_SPEED);
    x->downRight = LimitValue(x->downRight, 0.0f, MAX_SPEED);
    x->upLeft    = LimitValue(x->upLeft,    0.0f, MAX_SPEED);
    x->downLeft  = LimitValue(x->downLeft,  0.0f, MAX_SPEED);
}

void MotorsReset(Motors *x) {
    x->upRight   = 0.0;
    x->downLeft  = 0.0; 
    x->upLeft    = 0.0; 
    x->downRight = 0.0;
}

void CalibrateESC() {
    WriteRGBLed(0, 0, 4095);   
    delay_ms(5000); 
    WriteRGBLed(0, 4095, 4095); 

    write_pwm(MOTOR_UPRIGHT_PIN,   MOTOR_MAX);
    write_pwm(MOTOR_DOWNLEFT_PIN,  MOTOR_MAX);
    write_pwm(MOTOR_UPLEFT_PIN,    MOTOR_MAX);
    write_pwm(MOTOR_DOWNRIGHT_PIN, MOTOR_MAX);

     while(XBee.y2 > 5)
        delay_ms(10);
    
    TurnMotorsOff();
}

void WriteMotors(Motors x) {
    float ur = x.upRight   / MAX_SPEED * (MOTOR_MAX - MOTOR_OFF);
    float dl = x.downLeft  / MAX_SPEED * (MOTOR_MAX - MOTOR_OFF);
    float ul = x.upLeft    / MAX_SPEED * (MOTOR_MAX - MOTOR_OFF);
    float dr = x.downRight / MAX_SPEED * (MOTOR_MAX - MOTOR_OFF);

    write_pwm(MOTOR_UPRIGHT_PIN, MOTOR_OFF   + (int)ur);
    write_pwm(MOTOR_DOWNLEFT_PIN, MOTOR_OFF  + (int)dl);
    write_pwm(MOTOR_UPLEFT_PIN, MOTOR_OFF    + (int)ul);
    write_pwm(MOTOR_DOWNRIGHT_PIN, MOTOR_OFF + (int)dr); 
}

void TurnMotorsOff() {
    write_pwm(MOTOR_UPRIGHT_PIN,   MOTOR_OFF);
    write_pwm(MOTOR_DOWNLEFT_PIN,  MOTOR_OFF);
    write_pwm(MOTOR_UPLEFT_PIN,    MOTOR_OFF);
    write_pwm(MOTOR_DOWNRIGHT_PIN, MOTOR_OFF);
}
