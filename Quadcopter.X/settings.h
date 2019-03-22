#ifndef _settings_H_
#define _settings_H_

//-----------------------------PCB----------------------------
#define micro		//micro / mini / big
#define board_v3	//board_v1 / board_v2 / board_v3 / board_v4
//------------------------------------------------------------

//--------------------------Angles----------------------------
#define REMOTE_MAX 15.0f
#define THROTTLE_MAX 31.0f

#define MAX_PITCH_ROLL_TILT 20.0f   // degrees
#define MAX_YAW_RATE 180.0f         // degrees/sec
#define ALTITUDE_RATE 0.7f          // meters/sec

#define ROLLOFFSET 3.0f				//degrees
#define PITCHOFFSET -0.3			//degrees
#define HEADINGOFFSET -87           //degrees


//-------------------------Pinouts----------------------------
#ifdef board_v1
	#define RGBLED_RED_PIN 0
	#define RGBLED_GREEN_PIN 1
	#define RGBLED_BLUE_PIN 2
	#define MOTOR_UPRIGHT_PIN 6
	#define MOTOR_DOWNLEFT_PIN 4
	#define MOTOR_UPLEFT_PIN 3 
	#define MOTOR_DOWNRIGHT_PIN 5

	#define SCL PORTFbits.RF5
	#define SCL_TRIS TRISFbits.TRISF5
	#define SDA PORTFbits.RF4  
	#define SDA_TRIS TRISFbits.TRISF4
#endif
#ifdef board_v2
	#define RGBLED_RED_PIN 1
	#define RGBLED_GREEN_PIN 0
	#define RGBLED_BLUE_PIN 2
	#define MOTOR_UPRIGHT_PIN 6
	#define MOTOR_DOWNLEFT_PIN 4
	#define MOTOR_UPLEFT_PIN 3 
	#define MOTOR_DOWNRIGHT_PIN 5

	#define SCL PORTFbits.RF5
	#define SCL_TRIS TRISFbits.TRISF5
	#define SDA PORTFbits.RF4  
	#define SDA_TRIS TRISFbits.TRISF4
#endif
#ifdef board_v3
	#define RGBLED_RED_PIN 9
	#define RGBLED_GREEN_PIN 10
	#define RGBLED_BLUE_PIN 8
	#define MOTOR_UPRIGHT_PIN 3
	#define MOTOR_DOWNLEFT_PIN 1
	#define MOTOR_UPLEFT_PIN 0 
	#define MOTOR_DOWNRIGHT_PIN 2

	#define SCL PORTFbits.RF5
	#define SCL_TRIS TRISFbits.TRISF5
	#define SDA PORTFbits.RF4  
	#define SDA_TRIS TRISFbits.TRISF4
#endif
#ifdef board_v4
	#define RGBLED_RED_PIN 9		//RB14 - OC9
	#define RGBLED_GREEN_PIN 5		//RB15 - OC5
	#define RGBLED_BLUE_PIN 8		//RB7 - OC8
	#define MOTOR_UPRIGHT_PIN 1		//RG9 - OC1
	#define MOTOR_DOWNLEFT_PIN 4	//RB3 - OC4
	#define MOTOR_UPLEFT_PIN 2		//RB2 - OC2
	#define MOTOR_DOWNRIGHT_PIN 3	//RB5 - OC3

	#define SCL PORTFbits.RF5
	#define SCL_TRIS TRISFbits.TRISF5
	#define SDA PORTFbits.RF4  
	#define SDA_TRIS TRISFbits.TRISF4
#endif

//------------------------ESC timings--------------------------
#ifdef board_v4
	#define ESC_FREQ 10000.0f		//Hz
	#define MOTOR_OFF_TIME 42.0f	//us
	#define MOTOR_MAX_TIME 84.0f	//us
	
#else
	#define ESC_FREQ 470.0f			//Hz
	#define MOTOR_OFF_TIME 1000.0f	//us
	#define MOTOR_MAX_TIME 2000.0f	//us
#endif
#define ESC_TIME_us 1000000.0 / ESC_FREQ	//ESC update period in us

//Motor spin correction: +1 for top right motor CCW, -1 for top right motor CW
#ifdef micro
    #define MOTOR_SPIN -1
#endif
#ifdef mini
    #define MOTOR_SPIN 1
#endif
#ifdef big
    #define MOTOR_SPIN 1
#endif

#define MS5611	//BMP180 or MS5611
#define IMU_BUFFER_SIZE 10
#define ALTITUDE_BUFFER_SIZE 3

#define PI 3.14159265
#define RAD_TO_DEGREES 57.29577951

#endif