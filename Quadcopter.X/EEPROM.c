#include "EEPROM.h"
#include "bitbang_I2C.h"
#include "PID.h"
#include "pic32.h"
#include "10DOF.h"
#include "AHRS.h"
#include <stdbool.h>

#define EEPROM_ADDRESS  0xA0

#define EEPROM_INITIAL_ADDR 0x00
#define EEPROM_INITIAL_KEY  0x4A

#define P_ADDR      16
#define I_ADDR      32
#define D_ADDR      48
#define GPS_ADDR    64

#define COMPASS_X_OFFSET_ADDR  96
#define COMPASS_Y_OFFSET_ADDR  98
#define COMPASS_z_OFFSET_ADDR  100
#define COMPASS_X_GAIN_ADDR  102
#define COMPASS_Y_GAIN_ADDR  104
#define COMPASS_z_GAIN_ADDR  106

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

void eeprom_readPID(PID *roll, PID *pitch, PID *yaw, PID *alt, PID *gps) {
    unsigned char str[16];
    
    //Read previously saved data
    eeprom_readBytes(P_ADDR, str, 16);
    roll->p = *(float*)(unsigned char[4]){str[0], str[1], str[2], str[3]};
    pitch->p = *(float*)(unsigned char[4]){str[4], str[5], str[6], str[7]};
    yaw->p = *(float*)(unsigned char[4]){str[8], str[9], str[10], str[11]};
    alt->p = *(float*)(unsigned char[4]){str[12], str[13], str[14], str[15]};

    eeprom_readBytes(I_ADDR, str, 16);
    roll->i = *(float*)(unsigned char[4]){str[0], str[1], str[2], str[3]};
    pitch->i = *(float*)(unsigned char[4]){str[4], str[5], str[6], str[7]};
    yaw->i = *(float*)(unsigned char[4]){str[8], str[9], str[10], str[11]};
    alt->i = *(float*)(unsigned char[4]){str[12], str[13], str[14], str[15]};

    eeprom_readBytes(D_ADDR, str, 16);
    roll->d = *(float*)(unsigned char[4]){str[0], str[1], str[2], str[3]};
    pitch->d = *(float*)(unsigned char[4]){str[4], str[5], str[6], str[7]};
    yaw->d = *(float*)(unsigned char[4]){str[8], str[9], str[10], str[11]};
    alt->d = *(float*)(unsigned char[4]){str[12], str[13], str[14], str[15]};

    eeprom_readBytes(GPS_ADDR, str, 12);
    gps->p = *(float*)(unsigned char[4]){str[0], str[1], str[2], str[3]};
    gps->i = *(float*)(unsigned char[4]){str[4], str[5], str[6], str[7]};
    gps->d = *(float*)(unsigned char[4]){str[8], str[9], str[10], str[11]};
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
}

void eeprom_readCalibration() {
    unsigned char str[12];
    //Read previously saved data
    eeprom_readBytes(COMPASS_X_OFFSET_ADDR, str, 12);
    compass_offset.x = (float)*(signed short*)(unsigned char[2]){str[0], str[1]};
    compass_offset.y = (float)*(signed short*)(unsigned char[2]){str[2], str[3]};
    compass_offset.z = (float)*(signed short*)(unsigned char[2]){str[4], str[5]};
    compass_gain.x = (float)*(signed short*)(unsigned char[2]){str[6], str[7]};
    compass_gain.y = (float)*(signed short*)(unsigned char[2]){str[8], str[9]};
    compass_gain.z = (float)*(signed short*)(unsigned char[2]){str[10], str[11]};
}

void eeprom_writeCalibration() {
    unsigned char str[12];
    
    *(float*)(str) = (signed short)compass_offset.x;
    *(float*)(str + 2) = (signed short)compass_offset.y;
    *(float*)(str + 4) = (signed short)compass_offset.z;
    *(float*)(str + 6) = (signed short)compass_gain.x;
    *(float*)(str + 8) = (signed short)compass_gain.y;
    *(float*)(str + 10) = (signed short)compass_gain.z;
    eeprom_writeBytes(COMPASS_X_OFFSET_ADDR, str, 12);
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