#include "init.h"
#include "pic32.h"
#include "10DOF.h"
#include "USART.h"
#include "PWMDriver.h"
#include "PID.h"

void Init() {
    PICInit();
    timer2_init(1000);      // Delay timer - 1kHz
    timer3_init(1000);      // Safety timer for Xbee - 1kHz
    timer4_init(1000000);   // Loop timer - 1MHz
    timer5_init(1000);      // GPS timer - 1kHz
    timer6_init(312500);    // XBee tx timer - 312.5kHz
    USART1_init(111111);    // XBee
    USART5_init(9600);      // GPS

    //Initializing all devices: MPU6050, HMC5883, PWM driver and the four ESCs
    delay_ms(100);
    MPU6050Init();
    HMC5883Init();
    BMP180Init();
    
    PwmDriverInit(500);
}

void ResetPID(PID *roll, PID *pitch, PID *yaw, PID *roll_rate, PID *pitch_rate, PID *yaw_rate, PID *altitude, PID *altitude_rate, PID *GPS) {
    PIDSet(roll, roll->p, roll->i, roll->d);
    PIDSet(pitch, pitch->p, pitch->i, pitch->d);
    PIDSet(yaw, yaw->p, yaw->i, yaw->d);

    PIDSet(roll_rate, roll_rate->p, roll_rate->i, roll_rate->d);
    PIDSet(pitch_rate, pitch_rate->p, pitch_rate->i, pitch_rate->d);
    PIDSet(yaw_rate, yaw_rate->p, yaw_rate->i, yaw_rate->d);

    PIDSet(altitude, altitude->p, altitude->i, altitude->d);
    PIDSet(altitude_rate, altitude_rate->p, altitude_rate->i, altitude_rate->d);

    PIDSet(GPS, GPS->p, GPS->i, GPS->d);
}

void ResetQuaternion(float q[]){
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
}