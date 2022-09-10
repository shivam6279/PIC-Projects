#ifndef _SPI_H_
#define _SPI_H_

#include <xc.h>

extern void SPI_init();
extern void SPI_write(unsigned char);

extern void start_frame();
extern void end_frame();
extern void LED_frame(unsigned char, unsigned char, unsigned char);
extern void Write_Onboard_LEDs(unsigned char, unsigned char, unsigned char);

#endif


