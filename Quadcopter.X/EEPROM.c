#include "EEPROM.h"
#include "bitbang_I2C.h"
#include "PID.h"
#include "pic32.h"
#include <stdbool.h>

#define EEPROM_ADDRESS  0xA0

#define EEPROM_INITIAL_ADDRESS  0x00
#define EEPROM_INITIAL_KEY      0x3C

#define ROLL_ADDR       16
#define PITCH_ADDR      32
#define YAW_ADDR        48
#define ALTITUDE_ADDR   64
#define GPS_ADDR        80

#define COMPASS_X_MIN_ADDR  96
#define COMPASS_Y_MIN_ADDR  100
#define COMPASS_z_MIN_ADDR  104

#define COMPASS_X_MAX_ADDR  112
#define COMPASS_Y_MAX_ADDR  116
#define COMPASS_z_MAX_ADDR  120

#define GYRO_X_OFFSET_ADDR  128
#define GYRO_Y_OFFSET_ADDR  132
#define GYRO_Z_OFFSET_ADDR  136

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
    unsigned char str[12];
    
    if(eeprom_readByte(EEPROM_INITIAL_ADDRESS) == EEPROM_INITIAL_KEY) {
        //Read previously saved data
        eeprom_readBytes(ROLL_ADDR, str, 12);
        roll->p = *(float*)(unsigned char[4]){str[0], str[1], str[2], str[3]};
        roll->i = *(float*)(unsigned char[4]){str[4], str[5], str[6], str[7]};
        roll->d = *(float*)(unsigned char[4]){str[8], str[9], str[10], str[11]};

        eeprom_readBytes(PITCH_ADDR, str, 12);
        pitch->p = *(float*)(unsigned char[4]){str[0], str[1], str[2], str[3]};
        pitch->i = *(float*)(unsigned char[4]){str[4], str[5], str[6], str[7]};
        pitch->d = *(float*)(unsigned char[4]){str[8], str[9], str[10], str[11]};

        eeprom_readBytes(YAW_ADDR, str, 12);
        yaw->p = *(float*)(unsigned char[4]){str[0], str[1], str[2], str[3]};
        yaw->i = *(float*)(unsigned char[4]){str[4], str[5], str[6], str[7]};
        yaw->d = *(float*)(unsigned char[4]){str[8], str[9], str[10], str[11]};

        eeprom_readBytes(ALTITUDE_ADDR, str, 12);
        alt->p = *(float*)(unsigned char[4]){str[0], str[1], str[2], str[3]};
        alt->i = *(float*)(unsigned char[4]){str[4], str[5], str[6], str[7]};
        alt->d = *(float*)(unsigned char[4]){str[8], str[9], str[10], str[11]};

        eeprom_readBytes(GPS_ADDR, str, 12);
        gps->p = *(float*)(unsigned char[4]){str[0], str[1], str[2], str[3]};
        gps->i = *(float*)(unsigned char[4]){str[4], str[5], str[6], str[7]};
        gps->d = *(float*)(unsigned char[4]){str[8], str[9], str[10], str[11]};
    } 
}

void eeprom_writePID(PID *roll, PID *pitch, PID *yaw, PID *alt, PID *gps) {
    unsigned char str[12];
    
    //Write data
    *(float*)(str) = roll->p;
    *(float*)(str + 4) = roll->i;
    *(float*)(str + 8) = roll->d;
    eeprom_writeBytes(ROLL_ADDR, str, 12);
    delay_ms(6);

    *(float*)(str) = pitch->p;
    *(float*)(str + 4) = pitch->i;
    *(float*)(str + 8) = pitch->d;
    eeprom_writeBytes(PITCH_ADDR, str, 12);
    delay_ms(6);

    *(float*)(str) = yaw->p;
    *(float*)(str + 4) = yaw->i;
    *(float*)(str + 8) = yaw->d;
    eeprom_writeBytes(YAW_ADDR, str, 12);
    delay_ms(6);

    *(float*)(str) = alt->p;
    *(float*)(str + 4) = alt->i;
    *(float*)(str + 8) = alt->d;
    eeprom_writeBytes(ALTITUDE_ADDR, str, 12);
    delay_ms(6);
    
    *(float*)(str) = gps->p;
    *(float*)(str + 4) = gps->i;
    *(float*)(str + 8) = gps->d;
    eeprom_writeBytes(GPS_ADDR, str, 12);
    delay_ms(6);
    
    eeprom_writeByte(EEPROM_INITIAL_ADDRESS, EEPROM_INITIAL_KEY);
    delay_ms(6);
}