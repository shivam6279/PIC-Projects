#ifndef _I2C_H_
#define _I2C_H_

#include <stdbool.h>
#include <inttypes.h>

extern bool I2C_WriteRegisters(uint8_t, uint8_t*, uint16_t);
extern bool I2C_ReadRegisters(uint8_t, uint8_t, uint8_t*, uint16_t);
extern bool I2C_CheckAddress(uint8_t);
extern void I2C_Send(uint8_t);
extern uint8_t I2C_Read();
extern bool I2C_GetAck();
extern void I2C_SendAck();
extern void I2C_SendNak();
extern void I2C_Start();
extern void I2C_Stop();
extern void I2C_Restart();

extern bool I2C_CheckAddress(uint8_t);

#endif
