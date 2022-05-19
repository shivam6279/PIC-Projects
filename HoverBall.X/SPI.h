#ifndef _SPI_H_
#define _SPI_H_

#include <xc.h>

#define LED_TX_INTERRUPT IEC5bits.SPI4TXIE

extern void SPI_init();
extern void SPI_write(unsigned char);

#endif


