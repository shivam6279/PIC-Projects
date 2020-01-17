#ifndef _settings_H_
#define _settings_H_

//-----------------------------PCB----------------------------
#define micro		//micro / mini / big
#define board_version 5
//------------------------------------------------------------

//--------------------------Angles----------------------------
#define REMOTE_MAX 15.0f
#define THROTTLE_MAX 31.0f

#define MAX_PITCH_ROLL_TILT 20.0f   // degrees
#define MAX_YAW_RATE 180.0f         // degrees/sec
#define MAX_ALTITUDE_RATE 0.7f      // meters/sec

#define ROLLOFFSET -1.4f			// degrees
#define PITCHOFFSET 1.4f			// degrees
#define HEADINGOFFSET 0.0f       	// degrees

#define MIN_THROTTLE_INTEGRATION 3

#if board_version == 1
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

	#define ESC_FREQ 470.0f			//Hz
	#define MOTOR_OFF_TIME 1000.0f	//us
	#define MOTOR_MAX_TIME 2000.0f	//us

#elif board_version == 2
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

	#define ESC_FREQ 470.0f			//Hz
	#define MOTOR_OFF_TIME 1000.0f	//us
	#define MOTOR_MAX_TIME 2000.0f	//us

#elif board_version == 3
	#define RGBLED_RED_PIN 8
	#define RGBLED_GREEN_PIN 9
	#define RGBLED_BLUE_PIN 10
	#define MOTOR_UPRIGHT_PIN 3
	#define MOTOR_DOWNLEFT_PIN 1
	#define MOTOR_UPLEFT_PIN 0 
	#define MOTOR_DOWNRIGHT_PIN 2

	#define SCL PORTFbits.RF5
	#define SCL_TRIS TRISFbits.TRISF5
	#define SDA PORTFbits.RF4  
	#define SDA_TRIS TRISFbits.TRISF4

	#define ESC_FREQ 470.0f			//Hz
	#define MOTOR_OFF_TIME 1000.0f	//us
	#define MOTOR_MAX_TIME 2000.0f	//us

#elif board_version == 4
	#define RGBLED_RED_PIN 9		//RB14 - OC9
	#define RGBLED_GREEN_PIN 5		//RB15 - OC5
	#define RGBLED_BLUE_PIN 8		//RB7  - OC8
	#define MOTOR_UPRIGHT_PIN 1		//RG9  - OC1
	#define MOTOR_DOWNLEFT_PIN 4	//RB3  - OC4
	#define MOTOR_UPLEFT_PIN 2		//RB2  - OC2
	#define MOTOR_DOWNRIGHT_PIN 3	//RB5  - OC3

	#define SCL PORTDbits.RD9
	#define SCL_TRIS TRISDbits.TRISD9
	#define SDA PORTFbits.RF4  
	#define SDA_TRIS TRISFbits.TRISF4

	#define ESC_FREQ 10000.0f		//Hz
	#define MOTOR_OFF_TIME 42.0f	//us
	#define MOTOR_MAX_TIME 84.0f	//us

#elif board_version == 5
	#define RGBLED_RED_PIN 5		//RB14 - OC9
	#define RGBLED_GREEN_PIN 9		//RB15 - OC5
	#define RGBLED_BLUE_PIN 8		//RB7  - OC8
	#define MOTOR_UPRIGHT_PIN 1		//RG9  - OC1
	#define MOTOR_DOWNLEFT_PIN 4	//RB3  - OC4
	#define MOTOR_UPLEFT_PIN 2		//RB2  - OC2
	#define MOTOR_DOWNRIGHT_PIN 3	//RB5  - OC3

	#define SCL_PORT PORTFbits.RF5
	#define SCL_TRIS TRISFbits.TRISF5
    #define SCL_LAT LATFbits.LATF5
	#define SDA_PORT PORTFbits.RF4  
	#define SDA_TRIS TRISFbits.TRISF4
    #define SDA_LAT LATFbits.LATF4

	#define ESC_FREQ 10000.0f		//Hz
	#define MOTOR_OFF_TIME 42.0f	//us
	#define MOTOR_MAX_TIME 84.0f	//us
#endif

#define ESC_TIME_us 1000000.0 / ESC_FREQ	//ESC update period in us

//Motor spin correction: +1 for top right motor CCW, -1 for top right motor CW
#ifdef micro
    #define MOTOR_SPIN 1.0f
#endif
#ifdef mini
    #define MOTOR_SPIN 1.0f
#endif
#ifdef big
    #define MOTOR_SPIN 1.0f
#endif

//IMU'
#define QMC5883	//HMC5883 or QMC5883
#define MS5611	//BMP180 or MS5611

#define PI 3.1415926535897931f

#define TO_DEG(x) (x * 180.0 / PI)
#define TO_RAD(x) (x * PI / 180.0)

#endif