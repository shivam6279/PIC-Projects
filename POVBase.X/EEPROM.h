#ifndef _EEPROM_H
#define _EEPROM_H

#define ID_ADDR 0x04
#define GYRO_X_OFFSET_ADDR 0x08
#define GYRO_Y_OFFSET_ADDR 0x0C
#define GYRO_Z_OFFSET_ADDR 0x10
#define MOTOR_OFFSET_ADDR 0x14
#define ROLL_OFFSET_ADDR 0x18
#define ROLL_SETPOINT_ADDR 0x1C

extern void EEPROM_init();
extern unsigned int EEPROM_read();
extern void EEPROM_write(unsigned int, unsigned int);

extern void Write_Motor_Offset(float);
extern float Read_Motor_Offset();

#endif