#ifndef _settings_H_
#define _settings_H_

//--------------------------Angles----------------------------
#define REMOTE_MAX 15.0f
#define THROTTLE_MAX 31.0f

#define MAX_PITCH_ROLL_TILT 45.0f   // degrees
#define MAX_YAW_RATE 225.0f         // degrees/sec
#define MAX_ALTITUDE_RATE 0.7f      // meters/sec

#define ROLLOFFSET 0.0f             // degrees
#define PITCHOFFSET 0.0f			// degrees
#define HEADINGOFFSET 0.0f       	// degrees

#define MIN_THROTTLE_INTEGRATION 3

#define ANTI_WINDUP_MAX_ANGLE 7.5f  //degrees
#define ANTI_WINDUP_MAX_BOUND 100.0f//degrees

#define RGBLED_RED_PIN 6		//RB10 - OC6
#define RGBLED_GREEN_PIN 2		//RG9 - OC2
#define RGBLED_BLUE_PIN 4		//RG7  - OC4

#define MOTOR_UPLEFT_PIN 5		//RB8  - OC5
#define MOTOR_UPRIGHT_PIN 3		//RB9  - OC3
#define MOTOR_DOWNRIGHT_PIN 8	//RB7  - OC8
#define MOTOR_DOWNLEFT_PIN 1	//RB6  - OC1

#define SCL_PORT PORTFbits.RF5
#define SCL_TRIS TRISFbits.TRISF5
#define SCL_LAT LATFbits.LATF5
#define SDA_PORT PORTFbits.RF4  
#define SDA_TRIS TRISFbits.TRISF4
#define SDA_LAT LATFbits.LATF4

#define ESC_FREQ 10000.0f		//Hz
#define MOTOR_OFF_TIME 42.0f	//us
#define MOTOR_MAX_TIME 84.0f	//us

#define ESC_TIME_us 1000000.0 / ESC_FREQ	//ESC update period in us

#define USE_EEPROM 0
#define MOTOR_SPIN 1.0f

#define PI 3.1415926535897931f

#define TO_DEG(x) (x * 180.0 / PI)
#define TO_RAD(x) (x * PI / 180.0)

#endif