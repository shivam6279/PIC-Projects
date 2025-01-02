#ifndef _EEPROM_H
#define _EEPROM_H

#define ID_ADDR 0x04
#define MOTOR_OFFSET_ADDR 0x14
#define ENCODER_CALIB_ADDR 0x20

extern void EEPROM_init();
extern unsigned int EEPROM_read();
extern void EEPROM_write(unsigned int, unsigned int);

extern void Write_Motor_Offset(float);
extern float Read_Motor_Offset();

#endif
