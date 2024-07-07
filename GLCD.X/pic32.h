#ifndef _pic32_H_
#define _pic32_H_

extern void PICInit();

extern void StartDelayCounter();
extern void StopDelayCounter();
extern unsigned long int ms_counter();
extern void delay_ms(unsigned int);

extern void timer2_init(float);
extern void timer3_init(float);
extern void timer4_init(float);
extern void timer5_init(float);

extern volatile unsigned long int delay_counter;

#define DELAY_TIMER_ON T2CONbits.ON

#endif