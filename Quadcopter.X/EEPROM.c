#include "bitbang_I2C.h"
#include "PID.h"
#include <stdbool.h>

#define EEPROM_ADDRESS  0xA0

#define EEPROM_INITIAL_ADDRESS  0x00
#define EEPROM_INITIAL_KEY      0x3D

#define ROLL_ADDR       0x08
#define PITCH_ADDR      0x14
#define YAW_ADDR        0x20
#define ALTITUDE_ADDR   0x2C
#define GPS_ADDR        0x50

bool eeprom_writeByte(unsigned char addr, unsigned char byte) {
    I2C_Start();
    I2C_Send(EEPROM_ADDRESS & 0xFE); 
    if(!I2C_GetAck()) {
        I2C_Stop();
        return 0;
    }
    I2C_Send(byte);
    if(!I2C_GetAck()) {
        I2C_Stop();
        return 0;
    }
    I2C_Stop();
}

unsigned char eeprom_readByte(unsigned char addr) {
    unsigned char data;
    I2C_ReadRegisters(EEPROM_ADDRESS, addr, &data, 1);
    return data;
}

bool eeprom_writeBytes(unsigned char addr, unsigned char *bytes, unsigned char num) {
    unsigned char i, data[17];
    if(num > 16) 
        return 0;
    data[0] = addr;
    for(i = 0; i < (num + 1); i++) {
        data[i + 1] = bytes[i];
    }
    return I2C_WriteRegisters(EEPROM_ADDRESS, data, (num + 1));
}

unsigned char eeprom_readBytes(unsigned char addr, unsigned char *bytes, unsigned char num) {
    I2C_ReadRegisters(EEPROM_ADDRESS, addr, bytes, num)
    return data;
}

void eeprom_readPID(PID *roll, PID *pitch, PID *yaw, PID *alt, PID, *gps) {
    unsigned char key;
    unsigned char str[12];
    
    if(eeprom_readByte(EEPROM_INITIAL_ADDRESS) == EEPROM_INITIAL_KEY) {
        //Read previosly saved data
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

    } else {
        //Write data
    }
}