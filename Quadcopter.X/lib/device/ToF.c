#include <stdint.h>
#include <math.h>
#include "ToF.h"
#include "bitbang_I2C.h"
#include "pic32.h"

static uint8_t stop_variable;
static uint32_t measurement_timing_budget_us;

uint8_t buffer_start, buffer_end; 
float lidar_buffer_data[LIDAR_BUFFER_LEN];
uint8_t buffer_avg_coeffs[LIDAR_BUFFER_LEN] = {-2, -3, -6, 7, 6, 3, 2};
float buffer_delta_coeffs[LIDAR_BUFFER_LEN] = {-1, 9, -45, 0, 45, -9, 1};

void VL53L0X_init() {
    uint8_t read_byte;
    read_byte = VL53L0X_read8(0x89);
    VL53L0X_write8(0x89, read_byte | 0x01);

    // Set I2C standard mode
    VL53L0X_write8(0x88, 0x00);
    
    VL53L0X_write8(0x80, 0x01);
    VL53L0X_write8(0xFF, 0x01);
    VL53L0X_write8(0x00, 0x00);
    stop_variable = VL53L0X_read8(0x91);
    VL53L0X_write8(0x00, 0x01);
    VL53L0X_write8(0xFF, 0x00);
    VL53L0X_write8(0x80, 0x00);
    
    // Disable SIGNAL_RATE_MSRC
    read_byte = VL53L0X_read8(0x46);
    VL53L0X_write8(0x46, read_byte | 0x12);
    VL53L0X_write16(0x44, 32);
    VL53L0X_write8(0x01, 0xFF);
    
    // Load tuning settings
    VL53L0X_write8(0xFF, 0x01);
    VL53L0X_write8(0x00, 0x00);

    VL53L0X_write8(0xFF, 0x00);
    VL53L0X_write8(0x09, 0x00);
    VL53L0X_write8(0x10, 0x00);
    VL53L0X_write8(0x11, 0x00);

    VL53L0X_write8(0x24, 0x01);
    VL53L0X_write8(0x25, 0xFF);
    VL53L0X_write8(0x75, 0x00);

    VL53L0X_write8(0xFF, 0x01);
    VL53L0X_write8(0x4E, 0x2C);
    VL53L0X_write8(0x48, 0x00);
    VL53L0X_write8(0x30, 0x20);

    VL53L0X_write8(0xFF, 0x00);
    VL53L0X_write8(0x30, 0x09);
    VL53L0X_write8(0x54, 0x00);
    VL53L0X_write8(0x31, 0x04);
    VL53L0X_write8(0x32, 0x03);
    VL53L0X_write8(0x40, 0x83);
    VL53L0X_write8(0x46, 0x25);
    VL53L0X_write8(0x60, 0x00);
    VL53L0X_write8(0x27, 0x00);
    VL53L0X_write8(0x50, 0x06);
    VL53L0X_write8(0x51, 0x00);
    VL53L0X_write8(0x52, 0x96);
    VL53L0X_write8(0x56, 0x08);
    VL53L0X_write8(0x57, 0x30);
    VL53L0X_write8(0x61, 0x00);
    VL53L0X_write8(0x62, 0x00);
    VL53L0X_write8(0x64, 0x00);
    VL53L0X_write8(0x65, 0x00);
    VL53L0X_write8(0x66, 0xA0);

    VL53L0X_write8(0xFF, 0x01);
    VL53L0X_write8(0x22, 0x32);
    VL53L0X_write8(0x47, 0x14);
    VL53L0X_write8(0x49, 0xFF);
    VL53L0X_write8(0x4A, 0x00);

    VL53L0X_write8(0xFF, 0x00);
    VL53L0X_write8(0x7A, 0x0A);
    VL53L0X_write8(0x7B, 0x00);
    VL53L0X_write8(0x78, 0x21);

    VL53L0X_write8(0xFF, 0x01);
    VL53L0X_write8(0x23, 0x34);
    VL53L0X_write8(0x42, 0x00);
    VL53L0X_write8(0x44, 0xFF);
    VL53L0X_write8(0x45, 0x26);
    VL53L0X_write8(0x46, 0x05);
    VL53L0X_write8(0x40, 0x40);
    VL53L0X_write8(0x0E, 0x06);
    VL53L0X_write8(0x20, 0x1A);
    VL53L0X_write8(0x43, 0x40);

    VL53L0X_write8(0xFF, 0x00);
    VL53L0X_write8(0x34, 0x03);
    VL53L0X_write8(0x35, 0x44);

    VL53L0X_write8(0xFF, 0x01);
    VL53L0X_write8(0x31, 0x04);
    VL53L0X_write8(0x4B, 0x09);
    VL53L0X_write8(0x4C, 0x05);
    VL53L0X_write8(0x4D, 0x04);

    VL53L0X_write8(0xFF, 0x00);
    VL53L0X_write8(0x44, 0x00);
    VL53L0X_write8(0x45, 0x20);
    VL53L0X_write8(0x47, 0x08);
    VL53L0X_write8(0x48, 0x28);
    VL53L0X_write8(0x67, 0x00);
    VL53L0X_write8(0x70, 0x04);
    VL53L0X_write8(0x71, 0x01);
    VL53L0X_write8(0x72, 0xFE);
    VL53L0X_write8(0x76, 0x00);
    VL53L0X_write8(0x77, 0x00);

    VL53L0X_write8(0xFF, 0x01);
    VL53L0X_write8(0x0D, 0x01);

    VL53L0X_write8(0xFF, 0x00);
    VL53L0X_write8(0x80, 0x01);
    VL53L0X_write8(0x01, 0xF8);

    VL53L0X_write8(0xFF, 0x01);
    VL53L0X_write8(0x8E, 0x01);
    VL53L0X_write8(0x00, 0x01);
    VL53L0X_write8(0xFF, 0x00);
    VL53L0X_write8(0x80, 0x00);
    // End tuning load
    
    // GPIO config
    VL53L0X_write8(0x0A, 0x04);
    read_byte = VL53L0X_read8(0x84);
    VL53L0X_write8(0x84, read_byte & ~0x10);
    VL53L0X_write8(0x0B, 0x01);
    // GPIO config end
    
    //
    
    VL53L0X_write8(0x01, 0xE8);
    
    VL53L0X_write8(0x01, 0x01);
    VL53L0X_performSingleRefCalibration(0x40);
    VL53L0X_write8(0x01, 0x02);
    VL53L0X_performSingleRefCalibration(0x00);
    VL53L0X_write8(0x01, 0xE8);

    uint8_t i;
    for(i = 0; i < LIDAR_BUFFER_LEN; i++) {
        lidar_buffer_data[i] = 0;
    }
}

bool VL53L0X_performSingleRefCalibration(unsigned char vhv_init_byte) {
    VL53L0X_write8(0x00, 0x01 | vhv_init_byte);

    StartDelayCounter();
    while ((VL53L0X_read8(0x13) & 0x07) == 0) {
        if (ms_counter() > 500) {
            return false;
        }
    }
    VL53L0X_write8(0x0B, 0x01);
    VL53L0X_write8(0x00, 0x00);
    return true;
}

void VL53L0X_startContinuous(uint32_t period_ms) {
    VL53L0X_write8(0x80, 0x01);
    VL53L0X_write8(0xFF, 0x01);
    VL53L0X_write8(0x00, 0x00);
    VL53L0X_write8(0x91, stop_variable);
    VL53L0X_write8(0x00, 0x01);
    VL53L0X_write8(0xFF, 0x00);
    VL53L0X_write8(0x80, 0x00);
    
    VL53L0X_write8(0x00, 0x02);
}

uint16_t VL53L0X_readRange() {
    uint16_t range = VL53L0X_read16(0x14 + 10);
    VL53L0X_write8(0x0B, 0x01);
    VL53L0X_updateBuffer(range);

    return range;
}

void VL53L0X_updateBuffer(uint16_t range) {
    uint8_t i;
    
    if(range != 0 && range < 8000) {
        for(i = 1; i < LIDAR_BUFFER_LEN; i++) {
            lidar_buffer_data[i] = lidar_buffer_data[i-1];
        }
        lidar_buffer_data[0] = range;
    }
}

float VL53L0X_bufferAvg() {
    float range_avg, sum;
    uint8_t i;
    
    for(i = 0, range_avg = 0, sum = 0; i < LIDAR_BUFFER_LEN; i++) {
        range_avg += (float)buffer_avg_coeffs[i] * (float)lidar_buffer_data[i];
        sum += fabs(buffer_avg_coeffs[i]);
    }
    range_avg /= sum;
    return range_avg;
}

float VL53L0X_bufferDelta() {
    float range_delta, sum;
    uint8_t i;
    
    for(i = 0, range_delta = 0, sum = 0; i < LIDAR_BUFFER_LEN; i++) {
        range_delta += (float)buffer_delta_coeffs[i] * (float)lidar_buffer_data[i];
        //sum += fabs(buffer_delta_coeffs[i]);
    }
    range_delta /= 60.0f;//sum;
    return range_delta;
}

uint8_t VL53L0X_read8(uint8_t addr) {
    unsigned char data;
    
    I2C_Start();
    I2C_Send(VL6180X_ADDRESS << 1); 
    I2C_GetAck();
    I2C_Send(addr);
    I2C_GetAck();
    I2C_Start();
    I2C_Send((VL6180X_ADDRESS << 1) | 0x01); 
    I2C_GetAck();
    data = I2C_Read();
    I2C_SendNak();
    I2C_Stop();

    return data;
}

uint16_t VL53L0X_read16(uint8_t addr) {
    uint16_t data;
    
    I2C_Start();
    I2C_Send((VL6180X_ADDRESS << 1) & 0xFE); 
    I2C_GetAck();
    I2C_Send(addr);
    I2C_GetAck();
    I2C_Start();
    I2C_Send((VL6180X_ADDRESS << 1) | 0x01); 
    I2C_GetAck();
    data = ((unsigned short)I2C_Read() << 8) & 0xFF00;
    I2C_SendAck();
    data |= (unsigned short)I2C_Read();
    I2C_SendNak();
    I2C_Stop();
    
    return data;
}

void VL53L0X_write8(uint8_t addr, uint8_t data) {
    I2C_WriteRegisters(VL6180X_ADDRESS, (unsigned char[2]){addr, data}, 2);
//    delay_ms(1);
}

void VL53L0X_write16(uint8_t addr, uint16_t data) {
    I2C_WriteRegisters(VL6180X_ADDRESS, (unsigned char[3]){addr, (uint8_t)(data >> 8), (uint8_t)(data & 0xFF)}, 3);
//    delay_ms(1);
}