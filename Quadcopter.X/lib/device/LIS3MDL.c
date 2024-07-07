#include "LIS3MDL.h"
#include "bitbang_I2C.h"
#include <stdbool.h>
#include <math.h>
#include "pic32.h"

void LIS3MDL_Init() {    
    I2C_WriteRegisters(LIS3MDL_ADDR, (unsigned char[2]){0x21, 0x60}, 2);    
    I2C_WriteRegisters(LIS3MDL_ADDR, (unsigned char[2]){0x20, 0x7E}, 2);    
    I2C_WriteRegisters(LIS3MDL_ADDR, (unsigned char[2]){0x23, 0x0C}, 2);
    I2C_WriteRegisters(LIS3MDL_ADDR, (unsigned char[2]){0x24, 0x00}, 2);
    I2C_WriteRegisters(LIS3MDL_ADDR, (unsigned char[2]){0x22, 0x00}, 2);
    
//    I2C_WriteRegisters(LIS3MDL_ADDR, (unsigned char[2]){0x30, 0x05}, 2);
}

bool LIS3MDL_GetCompass(float *x, float *y, float *z) {
    unsigned char temp[6];
    
    if(!I2C_ReadRegisters(LIS3MDL_ADDR, 0x28, temp, 6))
        return false;

    // Order: XL, XH, YL, YH, ZL, ZH 
   
    *x = (signed short)(temp[1] << 8 | temp[0]);
    *y = (signed short)(temp[3] << 8 | temp[2]);
    *z = (signed short)(temp[5] << 8 | temp[4]);
    
    return true;
}