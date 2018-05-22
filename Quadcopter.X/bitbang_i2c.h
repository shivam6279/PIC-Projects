#ifndef _bitbang_i2c_H_
#define _bitbang_i2c_H_

#include <stdbool.h>

#define SCL3 PORTCbits.RC14
#define SCL3_TRIS TRISCbits.TRISC14
#define SDA3 PORTDbits.RD1
#define SDA3_TRIS TRISDbits.TRISD1

#define SCL5 PORTFbits.RF5
#define SCL5_TRIS TRISFbits.TRISF5
#define SDA5 PORTFbits.RF4  
#define SDA5_TRIS TRISFbits.TRISF4

extern void i2c3_write_registers(unsigned char address, unsigned char *data, unsigned int num);
extern unsigned char i2c3_read_registers(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num);
extern void i2c3_send(unsigned char byte);
extern unsigned char i2c3_read();
extern bool i2c3_getack();
extern void i2c3_sendack();
extern void i2c3_start();
extern void i2c3_stop();
extern void i2c3_clock();
extern unsigned char i2c3_readbit();

extern void i2c5_write_registers(unsigned char address, unsigned char *data, unsigned int num);
extern void i2c5_read_registers(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num);
extern void i2c5_send(unsigned char byte);
extern unsigned char i2c5_read();
extern bool i2c5_getack();
extern void i2c5_sendack();
extern void i2c5_sendnak();
extern void i2c5_start();
extern void i2c5_stop();
extern void i2c5_clock();
extern unsigned char i2c5_readbit();

extern inline i2c_delayhalf();
extern inline i2c_delayfull();
extern inline i2c_delaysettle();

#endif