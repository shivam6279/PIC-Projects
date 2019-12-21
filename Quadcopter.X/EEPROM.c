#include "EEPROM.h"
#include "bitbang_I2C.h"
#include "PID.h"
#include "pic32.h"
#include "10DOF.h"
#include "AHRS.h"
#include <stdbool.h>

#define EEPROM_ADDRESS  0xA0

#define EEPROM_INITIAL_ADDR 0x00
#define EEPROM_INITIAL_KEY  0xB5

#define P_ADDR      16
#define I_ADDR      32
#define D_ADDR      48
#define GPS_ADDR    64

#define COMPASS_X_MIN_ADDR  96
#define COMPASS_Y_MIN_ADDR  98
#define COMPASS_z_MIN_ADDR  100
#define COMPASS_X_MAX_ADDR  102
#define COMPASS_Y_MAX_ADDR  104
#define COMPASS_z_MAX_ADDR  106

#define GYRO_X_OFFSET_ADDR  112
#define GYRO_Y_OFFSET_ADDR  114
#define GYRO_Z_OFFSET_ADDR  116

#define ROLL_OFFSET_ADDR    128
#define PITCH_OFFSET_ADDR   132
#define HEADING_OFFSET_ADDR 136

bool eeprom_writeByte(unsigned char addr, unsigned char byte) {    
    I2C_WriteRegisters(EEPROM_ADDRESS, (unsigned char[2]){addr, byte}, 2);
}

unsigned char eeprom_readByte(unsigned char addr) {
    unsigned char data;
    
    I2C_Start();
    I2C_Send(EEPROM_ADDRESS & 0xFE); 
    I2C_GetAck();
    I2C_Send(addr);
    I2C_GetAck();
    I2C_Start();
    I2C_Send(EEPROM_ADDRESS | 0x01); 
    I2C_GetAck();
    data = I2C_Read();
    I2C_SendNak();
    I2C_Stop();
    
    return data;
}

bool eeprom_writeBytes(unsigned char addr, unsigned char *bytes, unsigned char num) {
    unsigned char i, data[17];
    if(num > 16) 
        return 0;
    data[0] = addr;
    for(i = 0; i < num; i++)
        data[i + 1] = bytes[i];
    
    return I2C_WriteRegisters(EEPROM_ADDRESS, data, (num + 1));
}

unsigned char eeprom_readBytes(unsigned char addr, unsigned char *bytes, unsigned char num) {
    I2C_ReadRegisters(EEPROM_ADDRESS, addr, bytes, num);
}

#define NaN(f) ( ((((unsigned char *)&f)[3] & 0x7f) == 0x7f ) && (((unsigned char *)&f)[2] & 0x80) )

bool eeprom_readPID(PID *roll, PID *pitch, PID *yaw, PID *alt, PID *gps) {
    unsigned char str[16], i;
    float val[4];
    
    //Read previously saved data
    if(eeprom_readByte(EEPROM_INITIAL_ADDR) == EEPROM_INITIAL_KEY) {
        eeprom_readBytes(P_ADDR, str, 16);
        for(i = 0; i < 4; i++) {
            val[i] = *(float*)(unsigned char[4]){str[i*4],  str[i*4+1],  str[i*4+2],  str[i*4+3]};
            if(NaN(val[i])) {
                return false; 
            }
        }
        roll->p  = val[0];
        pitch->p = val[1];
        yaw->p   = val[2];
        alt->p   = val[3];

        eeprom_readBytes(I_ADDR, str, 16);
        for(i = 0; i < 4; i++) {
            val[i] = *(float*)(unsigned char[4]){str[i*4],  str[i*4+1],  str[i*4+2],  str[i*4+3]};
            if(NaN(val[i])) {
                return false; 
            }
        }
        roll->i  = val[0];
        pitch->i = val[1];
        yaw->i   = val[2];
        alt->i   = val[3];

        eeprom_readBytes(D_ADDR, str, 16);
        for(i = 0; i < 4; i++) {
            val[i] = *(float*)(unsigned char[4]){str[i*4],  str[i*4+1],  str[i*4+2],  str[i*4+3]};
            if(NaN(val[i])) {
                return false; 
            }
        }
        roll->d  = val[0];
        pitch->d = val[1];
        yaw->d   = val[2];
        alt->d   = val[3];

        eeprom_readBytes(GPS_ADDR, str, 12);
        for(i = 0; i < 3; i++) {
            val[i] = *(float*)(unsigned char[4]){str[i*4],  str[i*4+1],  str[i*4+2],  str[i*4+3]};
            if(NaN(val[i])) {
                return false; 
            }
        }
        gps->p = val[0];
        gps->i = val[1];
        gps->d = val[2];
        
        return true;
    }    
    return false;
}

void eeprom_writePID(PID *roll, PID *pitch, PID *yaw, PID *alt, PID *gps) {
    unsigned char str[16];
    
    //Write data
    *(float*)(str) = roll->p;
    *(float*)(str + 4) = pitch->p;
    *(float*)(str + 8) = yaw->p;
    *(float*)(str + 12) = alt->p;
    eeprom_writeBytes(P_ADDR, str, 16);
    delay_ms(6);
    
    *(float*)(str) = roll->i;
    *(float*)(str + 4) = pitch->i;
    *(float*)(str + 8) = yaw->i;
    *(float*)(str + 12) = alt->i;
    eeprom_writeBytes(I_ADDR, str, 16);
    delay_ms(6);
    
    *(float*)(str) = roll->d;
    *(float*)(str + 4) = pitch->d;
    *(float*)(str + 8) = yaw->d;
    *(float*)(str + 12) = alt->d;
    eeprom_writeBytes(D_ADDR, str, 16);
    delay_ms(6);

    *(float*)(str) = gps->p;
    *(float*)(str + 4) = gps->i;
    *(float*)(str + 8) = gps->d;
    eeprom_writeBytes(GPS_ADDR, str, 12);
    delay_ms(6); 
    
    if(eeprom_readByte(EEPROM_INITIAL_ADDR) != EEPROM_INITIAL_KEY) {
        eeprom_writeByte(EEPROM_INITIAL_ADDR, EEPROM_INITIAL_KEY);
        delay_ms(6);
    }
}

bool eeprom_readCalibration() {
    unsigned char str[12];
    XYZ compass_min, compass_max;
    //Read previously saved data
    eeprom_readBytes(COMPASS_X_MIN_ADDR, str, 12); 
    compass_min.x = (float)(*(signed short*)(unsigned char[2]){str[0],  str[1]});
    compass_min.y = (float)(*(signed short*)(unsigned char[2]){str[2],  str[3]});
    compass_min.z = (float)(*(signed short*)(unsigned char[2]){str[4],  str[5]});
    compass_max.x   = (float)(*(signed short*)(unsigned char[2]){str[6],  str[7]});
    compass_max.y   = (float)(*(signed short*)(unsigned char[2]){str[8],  str[9]});
    compass_max.z   = (float)(*(signed short*)(unsigned char[2]){str[10], str[11]});
    ComputeCompassOffsetGain(compass_min, compass_max);
    
    return true;
}

void eeprom_writeCalibration(XYZ c_min, XYZ c_max) {
    unsigned char str[12];
    
    *(signed short*)(str) = (signed short)c_min.x;
    *(signed short*)(str + 2) = (signed short)c_min.y;
    *(signed short*)(str + 4) = (signed short)c_min.z;
    *(signed short*)(str + 6) = (signed short)c_max.x;
    *(signed short*)(str + 8) = (signed short)c_max.y;
    *(signed short*)(str + 10) = (signed short)c_max.z;
    eeprom_writeBytes(COMPASS_X_MIN_ADDR, str, 12);
    delay_ms(6);
}

void eeprom_readOffsets() {
    unsigned char str[12];
    
    if(eeprom_readByte(EEPROM_INITIAL_ADDR) == EEPROM_INITIAL_KEY) {
        //Read previously saved data
        eeprom_readBytes(ROLL_OFFSET_ADDR, str, 12);
        roll_offset = *(float*)(unsigned char[4]){str[0], str[1], str[2], str[3]};
        pitch_offset = *(float*)(unsigned char[4]){str[4], str[5], str[6], str[7]};
        heading_offset = *(float*)(unsigned char[4]){str[8], str[9], str[10], str[11]};
    } 
}

void eeprom_writeOffsets() {
    unsigned char str[12];
    
    *(float*)(str) = roll_offset;
    *(float*)(str + 4) = pitch_offset;
    *(float*)(str + 8) = heading_offset;
    eeprom_writeBytes(ROLL_OFFSET_ADDR, str, 12);
    delay_ms(6);
    
    if(eeprom_readByte(EEPROM_INITIAL_ADDR) != EEPROM_INITIAL_KEY) {
        eeprom_writeByte(EEPROM_INITIAL_ADDR, EEPROM_INITIAL_KEY);
        delay_ms(6);
    }
}