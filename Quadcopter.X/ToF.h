#ifndef _ToF_H_
#define _ToF_H_

#define VL6180X_ADDRESS 0x29

extern void VL6180_init();
extern unsigned char ToF_readRange();
extern unsigned char ToF_valueGood();
extern void VL6180_write8(unsigned int, unsigned char);
extern unsigned char VL6180_read8(unsigned int);

#endif