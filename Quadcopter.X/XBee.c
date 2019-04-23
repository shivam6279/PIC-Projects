#include <xc.h>
#include "XBee.h"
#include "pic32.h"
#include "10DOF.h"
#include "PID.h"
#include "GPS.h"
#include "USART.h"
#include "EEPROM.h"

volatile rx XBee, XBee_temp;

volatile int safety_counter = 0;
volatile unsigned int tx_buffer_index = 0;
volatile unsigned int tx_buffer_timer = 0;
volatile bool tx_flag = 0;
volatile bool XBee_signal_temp = 0;

volatile char tx_buffer[200];

void XBeeReset() {
    XBee.x1 = 0;
    XBee.y1 = 0;
    XBee.x2 = 0;
    XBee.y2 = 0;
    XBee.ls = 0;
    XBee.rs = 0;
    XBee.signal = 0;
    XBee.data_ready = 0;
    XBee_signal_temp = 0;
}

rx ReadXBee() {
    return XBee;
}

void SendCalibrationData() {   
    XYZ compass_min, compass_max;

    GetRawIMU();
    compass_max = compass;    
    compass_min = compass; 
    
    TX_TIMER_ON = 1;
    tx_buffer_index = 0;
    while(XBee.rs == 0) {
        GetRawIMU();
        if(compass.x > compass_max.x) compass_max.x = compass.x;
        if(compass.y > compass_max.y) compass_max.y = compass.y;
        if(compass.z > compass_max.z) compass_max.z = compass.z;
        
        if(compass.x < compass_min.x) compass_min.x = compass.x;
        if(compass.y < compass_min.y) compass_min.y = compass.y;
        if(compass.z < compass_min.z) compass_min.z = compass.z;
        
        tx_buffer[0] = 'D';
        StrWriteInt(compass.x, tx_buffer, 1);
        StrWriteInt(compass.y, tx_buffer, 8);
        StrWriteInt(compass.z, tx_buffer, 15);
        StrWriteInt(compass_min.x, tx_buffer, 22);
        StrWriteInt(compass_min.y, tx_buffer, 29);
        StrWriteInt(compass_min.z, tx_buffer, 36);
        StrWriteInt(compass_max.x, tx_buffer, 43);
        StrWriteInt(compass_max.y, tx_buffer, 50);
        StrWriteInt(compass_max.z, tx_buffer, 57);
        StrWriteInt(gyro.x, tx_buffer, 64);
        StrWriteInt(gyro.y, tx_buffer, 71);
        StrWriteInt(gyro.z, tx_buffer, 78);
        StrWriteInt(acc.x, tx_buffer, 85);
        StrWriteInt(acc.y, tx_buffer, 92);
        StrWriteInt(acc.z, tx_buffer, 99);
        tx_buffer[106] = '\r';
        tx_buffer[107] = '\0';
    
        tx_flag = 1;
        delay_ms(35);
    }

    ComputeCompassOffsetGain(compass_min, compass_max);

#if board_version == 4
    eeprom_writeCalibration();
#endif
}

void SendFlightData(PID roll, PID pitch, PID yaw, PID altitude, char loop_mode) { 
    tx_buffer[0] = 'C';
    StrWriteFloat(roll.error, 2, tx_buffer, 1);
    StrWriteFloat(pitch.error, 2, tx_buffer, 8);
    StrWriteFloat(yaw.error, 2, tx_buffer, 15);
    StrWriteFloat(altitude.error, 2, tx_buffer, 22);
    StrWriteFloat(altitude.output, 8, tx_buffer, 29);
    StrWriteFloat(yaw.output, 8, tx_buffer, 42);
    tx_buffer[55] = loop_mode;
    tx_buffer[56] = '\r';
    tx_buffer[57] = '\0';     
}

void XBee_writeBuffer() {
    for(tx_buffer_index = 0, tx_flag = 1; !U1STAbits.UTXBF && tx_buffer[tx_buffer_index] != '\0'; tx_buffer_index++)
        U1TXREG = tx_buffer[tx_buffer_index];
    if(tx_buffer[tx_buffer_index] == '\0')
        tx_flag = 0;
    else
        UART1_TX_INTERRUPT = 1;
}