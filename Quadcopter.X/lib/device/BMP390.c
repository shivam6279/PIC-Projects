#include "BMP390.h"
#include "bitbang_I2C.h"
#include <stdbool.h>
#include <math.h>
#include "pic32.h"

double bmp390_comp_t1, bmp390_comp_t2, bmp390_comp_p5, bmp390_comp_p6;
double bmp390_comp_p1, bmp390_comp_p2, bmp390_comp_p9;
double bmp390_comp_t3, bmp390_comp_p3, bmp390_comp_p4, bmp390_comp_p7, bmp390_comp_p8, bmp390_comp_p10, bmp390_comp_p11;

void BMP390_read_calibration_data() {
    unsigned char temp[21];
    I2C_ReadRegisters(BMP390_ADDR, 0x31, temp, 21);
    bmp390_comp_t1 = (float)((unsigned short)(temp[1] << 8) | temp[0]) / 0.00390625f;
    bmp390_comp_t2 = (float)((unsigned short)(temp[3] << 8) | temp[2]) / 1073741824.0f;
    bmp390_comp_t3 = (float)((signed char)temp[4]) / 281474976710656.0f;
    bmp390_comp_p1 = ((float)((signed short)((temp[6] << 8) | temp[5])) - 16384.0f) / 1048576.0f;
    bmp390_comp_p2 = ((float)((signed short)((temp[8] << 8) | temp[7])) - 16384.0f) / 536870912.0f;
    bmp390_comp_p3 = (float)((signed char)temp[9]) / 4294967296.0f;
    bmp390_comp_p4 = (float)((signed char)temp[10]) / 137438953472.0f;
    bmp390_comp_p5 = (float)((unsigned short)((temp[12] << 8) | temp[11])) / 0.125f;
    bmp390_comp_p6 = (float)((unsigned short)((temp[14] << 8) | temp[13])) / 64.0f;
    bmp390_comp_p7 = (float)((signed char)temp[15]) / 256.0f;
    bmp390_comp_p8 = (float)((signed char)temp[16]) / 32768.0f;
    bmp390_comp_p9 = (float)((unsigned short)((temp[18] << 8) | temp[17])) / 281474976710656.0f;
    bmp390_comp_p10 = (float)((signed char)temp[19]) / 281474976710656.0f;
    bmp390_comp_p11 = (float)((signed char)temp[20]) / 36893488147419103232.0f;
}

void BMP390_Init() {
    
    I2C_WriteRegisters(BMP390_ADDR, (unsigned char[2]){0x7E, 0xB6}, 2); // Soft reset
    delay_ms(100);
    
    I2C_WriteRegisters(BMP390_ADDR, (unsigned char[2]){0x17, 0b00000000}, 2); // Disable FIFO
    delay_ms(1);
    I2C_WriteRegisters(BMP390_ADDR, (unsigned char[2]){0x1B, 0b00110011}, 2); // Enable sensor, normal mode
    delay_ms(1);
    I2C_WriteRegisters(BMP390_ADDR, (unsigned char[2]){0x1C, 0b00001100}, 2); // OSR = x16
    delay_ms(1);
    I2C_WriteRegisters(BMP390_ADDR, (unsigned char[2]){0x1D, 4}, 2); // ODR = /4
    delay_ms(1);
    I2C_WriteRegisters(BMP390_ADDR, (unsigned char[2]){0x1F, 2}, 2); // IIR filter = 3
    delay_ms(1);
    BMP390_read_calibration_data();
}

unsigned long BMP390_read_pressure() {
    unsigned char temp[3];
    I2C_ReadRegisters(BMP390_ADDR, 0x04, temp, 3);
    return (unsigned long)((temp[2] << 16) | (temp[1] << 8) | temp[0]);
}

unsigned long BMP390_read_temp() {
    unsigned char temp[3];
    I2C_ReadRegisters(BMP390_ADDR, 0x07, temp, 3);
    return (unsigned long)((temp[2] << 16) | (temp[1] << 8) | temp[0]);
}

void BMP390_read_pressure_temp(unsigned long *pressure, unsigned long *temperature) {
    unsigned char temp[6];
    I2C_ReadRegisters(BMP390_ADDR, 0x04, temp, 6);
    *pressure = (temp[2] << 16) | (temp[1] << 8) | temp[0];
    *temperature = (temp[5] << 16) | (temp[4] << 8) | temp[3];
}

float BMP390_compensate_temp(float temp) {
    float pd1, pd2;
    pd1 = (temp - bmp390_comp_t1);
    pd2 = pd1 * bmp390_comp_t2;
    return pd2 + (pd1 * pd1) * bmp390_comp_t3;
}

float BMP390_compensate_pressure(float pressure, float comp_temp) {
    float pd1, pd2, pd3, pd4;
    float pout1, pout2;
    
    float comp_temp_2 = comp_temp * comp_temp;
    float comp_temp_3 = comp_temp_2 * comp_temp;
    
    pd1 = bmp390_comp_p6 * comp_temp;
    pd2 = bmp390_comp_p7 * comp_temp_2;
    pd3 = bmp390_comp_p8 * comp_temp_3;
    pout1 = bmp390_comp_p5 + pd1 + pd2 + pd3;
    
    pd1 = bmp390_comp_p2 * comp_temp;
    pd2 = bmp390_comp_p3 * comp_temp_2;
    pd3 = bmp390_comp_p4 * comp_temp_3;
    pout2 = pressure * (bmp390_comp_p1 + pd1 + pd2 + pd3);
    
    pd1 = pressure * pressure;
    pd2 = bmp390_comp_p9 + bmp390_comp_p10 * comp_temp;
    pd3 = pd1 * pd2;
    pd4 = pd3 + (pressure * pressure * pressure) * bmp390_comp_p11;
    
    return pout1 + pout2 + pd4;
}