#include "EEPROM.h"
#include <xc.h>
#include "MPU6050.h"

#define EEKEY1 0xEDB7
#define EEKEY2 0x1248

void EEPROM_init() {
    
    CFGCON2bits.EEWS = 5;
    
    EECONbits.ON = 1;

    while (EECONbits.RDY == 0);// Wait until EEPROM is ready (~125 us)

    EECONbits.WREN = 1;// Enable writing to the EEPROM
    EECONbits.CMD = 0b100;// Set the command to Configuration Write

    EEADDR = 0x00;// Addr 0x00 = DEVEE1;
    EEDATA = DEVEE0;
    EEKEY = EEKEY1; // Unlock the EEPROM to enable writing
    EEKEY = EEKEY2;
    EECONSET = _EECON_RW_MASK;
    while (EECONbits.RW); // desired

    EEADDR = 0x04;// Addr 0x04 = DEVEE2;
    EEDATA = DEVEE1;
    EEKEY = EEKEY1; // Unlock the EEPROM to enable writing
    EEKEY = EEKEY2;
    EECONSET = _EECON_RW_MASK;
    while (EECONbits.RW); // desired

    EEADDR = 0x08;// Addr 0x08 = DEVEE3;
    EEDATA = DEVEE2;
    EEKEY = EEKEY1; // Unlock the EEPROM to enable writing
    EEKEY = EEKEY2;
    EECONSET = _EECON_RW_MASK;
    while (EECONbits.RW); // desired

    EEADDR = 0x0C;// Addr 0x08 = DEVEE3;
    EEDATA = DEVEE3;
    EEKEY = EEKEY1; // Unlock the EEPROM to enable writing
    EEKEY = EEKEY2;
    EECONSET = _EECON_RW_MASK;
    while (EECONbits.RW); // desired

    EECONbits.WREN = 0; // Turn off writes.    
}

unsigned int EEPROM_read(unsigned int ee_addr) {
    unsigned int data;
    
    EEADDR = ee_addr & 0xFFC; // Set address on 32-bit boundary
    EECONbits.CMD = 0; // Load CMD<2:0> with
    // Data EEPROM read command
    EECONbits.WREN = 0; // Access for read

    EEKEY = EEKEY1; // Write unlock sequence
    EEKEY = EEKEY2;
    EECONSET = _EECON_RW_MASK; // Start the operation 

    while (EECONbits.RW==1); // Wait until read is complete
    data = EEDATA; // Read the data
    
    return data;
}

void EEPROM_write(unsigned int ee_addr, unsigned int ee_data)
{
    EECONbits.CMD = 1; // Load CMD<2:0> with write command
    EECONbits.WREN = 1; // Access for write

    EEADDR = ee_addr & 0xFFC; // Load address on a 32-bit boundary
    EEDATA = ee_data;

    EEKEY = EEKEY1; // Write unlock sequence
    EEKEY = EEKEY2;
    EECONSET = _EECON_RW_MASK; 

    while (EECONbits.RW == 1);
}

void CalibrateGyro() {
    XYZ gyro_avg = {0, 0, 0}, gyro;
    unsigned int i;
    
    for(i = 0 ; i < 1000; i++) {
        GetRawGyro(&gyro);
        gyro_avg.x += gyro.x;
        gyro_avg.y += gyro.y;
        gyro_avg.z += gyro.z;
        delay_ms(2);
    }
    gyro_avg.x /= 1000.0;
    gyro_avg.y /= 1000.0;
    gyro_avg.z /= 1000.0;
    
    EEPROM_write(GYRO_X_OFFSET_ADDR, *(unsigned int*)(char*)&gyro_avg.x);
    EEPROM_write(GYRO_Y_OFFSET_ADDR, *(unsigned int*)(char*)&gyro_avg.y);
    EEPROM_write(GYRO_Z_OFFSET_ADDR, *(unsigned int*)(char*)&gyro_avg.z);
}

XYZ ReadGyroCalibration() {
    XYZ gyro_offset;
    unsigned int temp;
    temp = EEPROM_read(GYRO_X_OFFSET_ADDR);
    gyro_offset.x = *(float*)(char*)&temp;
    
    temp = EEPROM_read(GYRO_Y_OFFSET_ADDR);
    gyro_offset.y = *(float*)(char*)&temp;
    
    temp = EEPROM_read(GYRO_Z_OFFSET_ADDR);
    gyro_offset.z = *(float*)(char*)&temp;
    
    return gyro_offset;
}

void Write_Motor_Offset(float off) {
    EEPROM_write(MOTOR_OFFSET_ADDR, *(unsigned int*)(char*)&off);
}

float Read_Motor_Offset() {
    unsigned int temp;
    temp = EEPROM_read(MOTOR_OFFSET_ADDR);
    return *(float*)(char*)&temp;
}

void Write_Roll_Offset(float off) {
    EEPROM_write(ROLL_OFFSET_ADDR, *(unsigned int*)(char*)&off);
}

float Read_Roll_Offset() {
    unsigned int temp;
    temp = EEPROM_read(ROLL_OFFSET_ADDR);
    return *(float*)(char*)&temp;
}

void Write_Roll_Setpoint(float off) {
    EEPROM_write(ROLL_SETPOINT_ADDR, *(unsigned int*)(char*)&off);
}

float Read_Roll_Setpoint() {
    unsigned int temp;
    temp = EEPROM_read(ROLL_SETPOINT_ADDR);
    return *(float*)(char*)&temp;
}