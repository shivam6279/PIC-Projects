#ifndef _settings_H_
#define _settings_H_

//-----------------------------PCB----------------------------
#define mini		//micro / mini / big
#define board_v4	//board_v1 / board_v2 / board_v3 / board_v4
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
	#define RGBLED_RED_PIN 9		//LATBbits.RB14	- OC9
	#define RGBLED_GREEN_PIN 5		//LATBbits.RB15	- OC5
	#define RGBLED_BLUE_PIN 8		//LATBbits.RB7 - OC8
	#define MOTOR_UPRIGHT_PIN 1		//LATGbits.RG9 - OC1
	#define MOTOR_DOWNLEFT_PIN 4	//LATBbits.RB3 - OC4
	#define MOTOR_UPLEFT_PIN 2		//LATBbits.RB2 - OC2
	#define MOTOR_DOWNRIGHT_PIN 3	//LATBbits.RB5 - OC3

	#define SCL PORTFbits.RF5
	#define SCL_TRIS TRISFbits.TRISF5
	#define SDA PORTFbits.RF4  
	#define SDA_TRIS TRISFbits.TRISF4
#endif

//------------------------ESC timings--------------------------
#ifndef board_v4
	#define ESC_FREQ 470f	//Hz
	#define MOTOR_OFF 1000f	//us
	#define MOTOR_MAX 2000f //us
#else
	#define ESC_FREQ 10000f	//Hz
	#define MOTOR_OFF 42.0f	//us
	#define MOTOR_MAX 84.0f	//us
#endif

#define MAX_SPEED 1000.0f

#define MS5611	//BMP180 or MS5611
#define IMU_BUFFER_SIZE 10
#define ALTITUDE_BUFFER_SIZE 3

#define PI 3.14159265
#define RAD_TO_DEGREES 57.29577951

#endif