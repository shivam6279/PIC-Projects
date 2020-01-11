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
    WriteRGBLed(0, 0, 255);   
    delay_ms(5000); 
    WriteRGBLed(0, 255, 255); 

    write_pwm(MOTOR_UPRIGHT_PIN,   MOTOR_MAX);
    write_pwm(MOTOR_DOWNLEFT_PIN,  MOTOR_MAX);
    write_pwm(MOTOR_UPLEFT_PIN,    MOTOR_MAX);
    write_pwm(MOTOR_DOWNRIGHT_PIN, MOTOR_MAX);

     while(XBee.y2 > 5)
        delay_ms(10);
    
    TurnMotorsOff();
}

void WriteMotors(Motors x) {
    float f = (MOTOR_MAX - MOTOR_OFF) / MAX_SPEED;

    write_pwm(MOTOR_UPRIGHT_PIN,   MOTOR_OFF + (int)(x.upRight   * f));
    write_pwm(MOTOR_DOWNLEFT_PIN,  MOTOR_OFF + (int)(x.downLeft  * f));
    write_pwm(MOTOR_UPLEFT_PIN,    MOTOR_OFF + (int)(x.upLeft    * f));
    write_pwm(MOTOR_DOWNRIGHT_PIN, MOTOR_OFF + (int)(x.downRight * f));
}

void TurnMotorsOff() {
    write_pwm(MOTOR_UPRIGHT_PIN,   MOTOR_OFF);
    write_pwm(MOTOR_DOWNLEFT_PIN,  MOTOR_OFF);
    write_pwm(MOTOR_UPLEFT_PIN,    MOTOR_OFF);
    write_pwm(MOTOR_DOWNRIGHT_PIN, MOTOR_OFF);
}
