#include "init.h"
#include "pic32.h"
#include "10DOF.h"
#include "USART.h"
#include "PWM.h"
#include "motor.h"
#include "PID.h"
#include "XBee.h"
#include "ToF.h"
#include "settings.h"
#include "SPI.h"

void Init() {
    PICInit();
    
    XBeeReset();
    USART4_init(111111);    // XBee
    timer7_init(1000.0);    // Safety timer for Xbee - 1kHz
    SAFETY_TIMER_ON = 1;
    
    timer2_init(1000.0);    // Delay timer - 1kHz    
    timer4_init(1000000.0); // Loop timer - 1MHz
    timer5_init(10.0);      // GPS timer - 10Hz
    timer6_init(312500.0);  // XBee tx timer - 312.5kHz
    
    USART5_init(9600);      // GPS
    
    pwm_init(ESC_FREQ);
    
    SPI_init();
}

void Init_10DOF(){
    MPU6050_Init();
    LIS3MDL_Init();
//    BMP180_Init();
}

void ResetQuaternion(float q[]){
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
}