#ifndef _SPI_H_
#define _SPI_H_

#include <xc.h>

#define LED_A_TX_INTERRUPT IEC5bits.SPI4TXIE
#define LED_B_TX_INTERRUPT IEC4bits.SPI3TXIE
#define LED_C_TX_INTERRUPT IEC4bits.SPI2TXIE

extern void SPI2_init();
extern void SPI3_init();
extern void SPI4_init();

extern void SPI2_write(unsigned char);
extern void SPI3_write(unsigned char);
extern void SPI4_write(unsigned char);

extern void SPI_all_write(unsigned char, unsigned char, unsigned char);

#endif
