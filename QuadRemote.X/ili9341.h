#ifndef _ili9341_H_
#define _ili9341_H_

#include <xc.h>

#define RST LATDbits.LATD4
#define DC LATDbits.LATD3
#define CS LATDbits.LATD2

extern void ColorLCD_writecommand(unsigned char);
extern void ColorLCD_writedata(unsigned char);
extern void ColorLCD_setxy(unsigned int, unsigned int, unsigned int, unsigned int);
extern void ColorLCD_init();

extern const unsigned char characters[][5];

#endif