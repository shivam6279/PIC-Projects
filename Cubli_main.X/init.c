#include "init.h"
#include "pic32.h"
#include "MPU9250.h"
#include "USART.h"
#include "PID.h"
#include "XBee.h"

void Init() {
    
}

void Init_10DOF(){
    MPU6050Init();
#ifdef HMC5883
    HMC5883Init();
#endif
#ifdef QMC5883
    QMC5883Init();
#endif
#ifdef BMP180
    BMP180Init();
#endif
#ifdef MS5611
    MS5611Init();
#endif
}

