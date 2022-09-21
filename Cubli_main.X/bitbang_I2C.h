#ifndef _bitbang_I2C_H_
#define _bitbang_I2C_H_

#include <stdbool.h>
#include <xc.h>

#define SDA_LAT LATFbits.LATF4
#define SDA_PORT PORTFbits.RF4
#define SDA_TRIS TRISFbits.TRISF4

#define SCL_LAT LATFbits.LATF5
#define SCL_PORT PORTFbits.RF5
#define SCL_TRIS TRISFbits.TRISF5

extern bool I2C_WriteRegisters(unsigned char address, unsigned char *data, unsigned int num);
extern bool I2C_ReadRegisters(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num);
extern bool I2C_CheckAddress(unsigned char);
extern void I2C_Send(unsigned char byte);
extern unsigned char I2C_Read();
extern bool I2C_GetAck();
extern void I2C_SendAck();
extern void I2C_SendNak();
extern void I2C_Start();
extern void I2C_Stop();
extern void I2C_Clock();
extern unsigned char I2C_ReadBit();

static inline void I2C_DelayHalf();
static inline void I2C_DelayFull();
static inline void I2C_DelaySettle();

#endif