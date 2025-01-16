#ifndef _SPI_H_
#define _SPI_H_

#include <xc.h>

extern void SPI2_init();
extern void SPI3_init();
extern void SPI4_init();

extern void SPI2_write(unsigned char);
extern void SPI3_write(unsigned char);
extern void SPI4_write(unsigned char);

extern void SPI_all_write(unsigned char, unsigned char, unsigned char);

#endif
