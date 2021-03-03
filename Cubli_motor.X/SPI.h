#ifndef _SPI_H_
#define _SPI_H_

#include <Stdbool.h>

extern void SPI_init();
extern unsigned char SPI_write(unsigned char data);
extern unsigned short SPI_write16(unsigned short data);

extern void A1339_init();
extern void A1339_test();
extern unsigned short A1339_read(unsigned short, unsigned short*);
extern unsigned short A1339_write(unsigned short, unsigned short);
extern unsigned int A1339_write32(unsigned short, unsigned int val);
extern bool CalculateParity(unsigned short);
extern unsigned char CalculateCRC(unsigned short);

#endif


