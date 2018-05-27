#ifndef _settings_H_
#define _settings_H_

//-----------------------------PCB----------------------------
#define big
//------------------------------------------------------------

#define REMOTE_MAX 15.0f
#define THROTTLE_MAX 31.0f

#define MAX_PITCH_ROLL_TILT 20.0f   // degrees
#define MAX_YAW_RATE 180.0f         // degrees/sec
#define ALTITUDE_RATE 0.7f          // meters/sec

#define ROLLOFFSET 0                //degrees
#define PITCHOFFSET 0               //degrees
#define HEADINGOFFSET -87           //degrees

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

#define MS5611 //BMP180 or MS5611
#define ALTITUDE_BUFFER_SIZE 3

#define PI 3.14159265
#define RAD_TO_DEGREES 57.29577951

#endif