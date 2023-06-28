#ifndef _bitbang_i2c_H_
#define _bitbang_i2c_H_

#include <xc.h>
#include <stdbool.h>

#define SCL3 PORTDbits.RD3 // PORTFbits.RF5
#define SCL3_TRIS TRISDbits.TRISD3 // TRISFbits.TRISF5
#define SDA3 PORTDbits.RD2 // PORTFbits.RF4  
#define SDA3_TRIS TRISDbits.TRISD2 // TRISFbits.TRISF4

#define SCL5 PORTFbits.RF5
#define SCL5_TRIS TRISFbits.TRISF5
#define SDA5 PORTFbits.RF4  
#define SDA5_TRIS TRISFbits.TRISF4

extern void i2c3_write_registers(unsigned char, unsigned char*, unsigned int);
extern unsigned char i2c3_read_registers(unsigned char, unsigned char, unsigned char*, unsigned int);
extern void i2c3_send(unsigned char byte);
extern unsigned char i2c3_read();
extern bool i2c3_getack();
extern void i2c3_sendack();
extern void i2c3_start();
extern void i2c3_stop();
extern void i2c3_clock();
extern unsigned char i2c3_readbit();

extern void i2c5_write_registers(unsigned char, unsigned char*, unsigned int);
extern void i2c5_read_registers(unsigned char, unsigned char, unsigned char*, unsigned int);
extern void i2c5_send(unsigned char byte);
extern unsigned char i2c5_read();
extern bool i2c5_getack();
extern void i2c5_sendack();
extern void i2c5_sendnak();
extern void i2c5_start();
extern void i2c5_stop();
extern void i2c5_clock();
extern unsigned char i2c5_readbit();

extern void i2c_delayhalf();
extern void i2c_delayfull();
extern void i2c_delaysettle();

#endif