#ifndef _pic32_H_
#define _pic32_H_

extern void PICInit();
extern void delay_ms(unsigned int);
extern void timer2_init(float);
extern void timer3_init(float);
extern void timer4_init(float);
extern void timer5_init(float);
extern void timer6_init(float);

unsigned long int delay_counter = 0;

#endif