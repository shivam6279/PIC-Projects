#ifndef _settings_H_
#define _settings_H_

//-----------------------------PCB----------------------------
#define big
//------------------------------------------------------------

#ifdef micro
#define RGBLED_RED_PIN 0
#define RGBLED_GREEN_PIN 1
#define RGBLED_BLUE_PIN 2
#define MOTOR_UPRIGHT_PIN 6
#define MOTOR_DOWNLEFT_PIN 4
#define MOTOR_UPLEFT_PIN 3 
#define MOTOR_DOWNRIGHT_PIN 5
#endif
#ifdef mini
#define RGBLED_RED_PIN 1
#define RGBLED_GREEN_PIN 0
#define RGBLED_BLUE_PIN 2
#define MOTOR_UPRIGHT_PIN 6
#define MOTOR_DOWNLEFT_PIN 4
#define MOTOR_UPLEFT_PIN 3 
#define MOTOR_DOWNRIGHT_PIN 5
#endif
#ifdef big
#define RGBLED_RED_PIN 9
#define RGBLED_GREEN_PIN 10
#define RGBLED_BLUE_PIN 8
#define MOTOR_UPRIGHT_PIN 3
#define MOTOR_DOWNLEFT_PIN 1
#define MOTOR_UPLEFT_PIN 0 
#define MOTOR_DOWNRIGHT_PIN 2
#endif

#endif