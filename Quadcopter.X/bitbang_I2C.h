#ifndef _bitbang_I2C_H_
#define _bitbang_I2C_H_

#include <stdbool.h>

#define SCL3 PORTCbits.RC14
#define SCL3_TRIS TRISCbits.TRISC14
#define SDA3 PORTDbits.RD1
#define SDA3_TRIS TRISDbits.TRISD1

#define SCL5 PORTFbits.RF5
#define SCL5_TRIS TRISFbits.TRISF5
#define SDA5 PORTFbits.RF4  
#define SDA5_TRIS TRISFbits.TRISF4

extern void I2C3_WriteRegisters(unsigned char address, unsigned char *data, unsigned int num);
extern unsigned char I2C3_ReadRegisters(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num);
extern void I2C3_Send(unsigned char byte);
extern unsigned char I2C3_read();
extern bool I2C3_GetAck();
extern void I2C3_SendAck();
extern void I2C3_Start();
extern void I2C3_Stop();
extern void I2C3_Clock();
extern unsigned char I2C3_ReadBit();

extern void I2C5_WriteRegisters(unsigned char address, unsigned char *data, unsigned int num);
extern void I2C5_ReadRegisters(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num);
extern void I2C5_Send(unsigned char byte);
extern unsigned char I2C5_Read();
extern bool I2C5_GetAck();
extern void I2C5_SendAck();
extern void I2C5_SendNak();
extern void I2C5_Start();
extern void I2C5_Stop();
extern void I2C5_Clock();
extern unsigned char I2C5_ReadBit();

extern inline I2C_DelayHalf();
extern inline I2C_DelayFull();
extern inline I2C_DelaySettle();

#endif