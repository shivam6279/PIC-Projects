#include "QMC5883.h"
#include "bitbang_I2C.h"
#include <stdbool.h>
#include <math.h>
#include "pic32.h"

void QMC5883_Init() {
    I2C_WriteRegisters(QMC5883_ADDR, (unsigned char[2]){0x0B, 0x01}, 2);

    I2C_WriteRegisters(QMC5883_ADDR, (unsigned char[2]){0x09, 0b01000101}, 2);
    I2C_WriteRegisters(QMC5883_ADDR, (unsigned char[2]){0x0A, 0b01000000}, 2);
}

bool QMC5883_GetCompass(float *x, float *y, float *z) {
    unsigned char temp[6];
    
    if(!I2C_ReadRegisters(QMC5883_ADDR, 0x00, temp, 6))
        return false;

    // Order: XL, XH, YL, YH, ZL, ZH 
   
    *x = (signed short)(temp[1] << 8 | temp[0]);
    *y = (signed short)(temp[3] << 8 | temp[2]);
    *z = (signed short)(temp[5] << 8 | temp[4]);
    
    return true;
}