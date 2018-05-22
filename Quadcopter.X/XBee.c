#include "XBee.h"
#include "10DOF.h"
#include "PID.h"
#include "GPS.h"
#include "USART.h"

void SendCalibrationData() {
    GetRawIMU();
    if(tx_buffer_index == 0) {     
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
        tx_buffer[64] = '\r';
        tx_buffer[65] = '\0';
        tx_buffer_index++;
    }
}

void SendFlightData(PID roll, PID pitch, PID yaw, PID altitude, char loop_mode) { 
    tx_buffer[0] = 'C';
    StrWriteFloat(roll.error, 3, 2, tx_buffer, 1);
    StrWriteFloat(pitch.error, 3, 2, tx_buffer, 8);
    StrWriteFloat(yaw.error, 3, 2, tx_buffer, 15);
    StrWriteFloat(altitude.error, 3, 2, tx_buffer, 22);
    StrWriteFloat(latitude, 3, 8, tx_buffer, 29);
    StrWriteFloat(longitude, 3, 8, tx_buffer, 42);
    tx_buffer[55] = loop_mode;
    tx_buffer[56] = '\r';
    tx_buffer[57] = '\0';     
    //USART1_send_str(tx_buffer);
}