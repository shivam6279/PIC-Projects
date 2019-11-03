#ifndef _SPI_H_
#define _SPI_H_

#include <xc.h>

extern void SPI_init();
extern void SPI4_init();

extern void SPI_write(unsigned char);
extern void SPI4_write(unsigned char);

#endif


