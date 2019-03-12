#include "XBee.h"
#include "pic32.h"
#include "10DOF.h"
#include "PID.h"
#include "GPS.h"
#include "USART.h"

volatile int remote_x1 = 0, remote_y1 = 0, remote_x2 = 0, remote_y2 = 0;
volatile int safety_counter = 0;

volatile unsigned char dial1, dial2;
volatile unsigned char receive1;
volatile unsigned char tx_buffer_index = 0;
volatile bool tx_flag = 0;

volatile bool left_switch = 0, right_switch = 0;
volatile bool Xbee_signal = 0;

char tx_buffer[200];

void SendCalibrationData() {
    XYZ compass_max, compass_min;
    
    GetRawIMU();
    compass_max.x = compass.x;
    compass_max.y = compass.y;
    compass_max.z = compass.z;
    
    compass_min.x = compass.x;
    compass_min.y = compass.y;
    compass_min.z = compass.z;    
    
    DELAY_TIMER_ON = 1;
    TX_TIMER_ON = 1;
    tx_buffer_index = 0;
    while(1) {
        GetRawIMU();
        if(compass.x > compass_max.x) compass_max.x = compass.x;
        if(compass.y > compass_max.y) compass_max.y = compass.y;
        if(compass.z > compass_max.z) compass_max.z = compass.z;
        
        if(compass.x < compass_min.x) compass_min.x = compass.x;
        if(compass.y < compass_min.y) compass_min.y = compass.y;
        if(compass.z < compass_min.z) compass_min.z = compass.z;
        
        tx_buffer[0] = 'D';
        StrWriteInt(acc.x, 6, tx_buffer, 1);
        StrWriteInt(acc.y, 6, tx_buffer, 8);
        StrWriteInt(acc.z, 6, tx_buffer, 15);
        StrWriteInt(gyro.x, 6, tx_buffer, 22);
        StrWriteInt(gyro.y, 6, tx_buffer, 29);
        StrWriteInt(gyro.z, 6, tx_buffer, 36);
        StrWriteInt(compass.x, 6, tx_buffer, 43);
        StrWriteInt(compass.y, 6, tx_buffer, 50);
        StrWriteInt(compass.z, 6, tx_buffer, 57);
        StrWriteInt(compass_max.x, 6, tx_buffer, 64);
        StrWriteInt(compass_max.y, 6, tx_buffer, 71);
        StrWriteInt(compass_max.z, 6, tx_buffer, 78);
        StrWriteInt(compass_min.x, 6, tx_buffer, 85);
        StrWriteInt(compass_min.y, 6, tx_buffer, 92);
        StrWriteInt(compass_min.z, 6, tx_buffer, 99);
        tx_buffer[106] = '\r';
        tx_buffer[107] = '\0';
    
        tx_flag = 1;
        delay_counter = 0;
        while(delay_counter < 25);
    }
}

void SendFlightData(PID roll, PID pitch, PID yaw, PID altitude, char loop_mode) { 
    tx_buffer[0] = 'C';
    StrWriteFloat(roll.error, 3, 2, tx_buffer, 1);
    StrWriteFloat(pitch.error, 3, 2, tx_buffer, 8);
    StrWriteFloat(yaw.error, 3, 2, tx_buffer, 15);
    StrWriteFloat(roll.output, 3, 2, tx_buffer, 22);
    StrWriteFloat(pitch.output, 3, 8, tx_buffer, 29);
    StrWriteFloat(yaw.output, 3, 8, tx_buffer, 42);
    tx_buffer[55] = loop_mode;
    tx_buffer[56] = '\r';
    tx_buffer[57] = '\0';     
}