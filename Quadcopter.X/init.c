#include "init.h"
#include "pic32.h"
#include "10DOF.h"
#include "USART.h"
#include "PWM.h"
#include "motor.h"
#include "PID.h"
#include "settings.h"

void Init() {
    PICInit();
    timer2_init(1000.0);    // Delay timer - 1kHz
    timer7_init(1000.0);    // Safety timer for Xbee - 1kHz
    timer4_init(1000000.0); // Loop timer - 1MHz
    timer5_init(10.0);      // GPS timer - 10Hz
    timer6_init(312500.0);  // XBee tx timer - 312.5kHz
    USART1_init(111111);    // XBee
    #ifdef board_v4
        USART3_init(9600);      // GPS
    #else
        USART5_init(9600);      // GPS
    #endif
        
    //Initializing all devices: MPU6050, HMC5883, PWM driver and the four ESCs
    delay_ms(100);
    MPU6050Init();
    HMC5883Init();
    #ifdef BMP180
        BMP180Init();
    #endif
    #ifdef MS5611
        MS5611Init();
    #endif
    
    pwm_init(ESC_FREQ);
}

void ResetPID(PID *roll, PID *pitch, PID *yaw, PID *altitude, PID *GPS) {
    PIDSet(roll, roll->p, roll->i, roll->d);
    PIDSet(pitch, pitch->p, pitch->i, pitch->d);
    PIDSet(yaw, yaw->p, yaw->i, yaw->d);

    PIDSet(altitude, altitude->p, altitude->i, altitude->d);

    PIDSet(GPS, GPS->p, GPS->i, GPS->d);
}

void ResetQuaternion(float q[]){
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
}