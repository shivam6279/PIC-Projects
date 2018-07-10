#ifndef _pic32_H_
#define _pic32_H_

#include <xc.h>

#define DELAY_TIMER_ON T2CONbits.ON
#define SAFETY_TIMER_ON T3CONbits.ON
#define LOOP_TIMER_ON T4CONbits.ON 
#define GPS_TIMER_ON T5CONbits.ON 
#define TX_TIMER_ON T6CONbits.ON

extern void PICInit();
extern void delay_ms(unsigned int);
extern void timer2_init(float);
extern void timer3_init(float);
extern void timer4_init(float);
extern void timer5_init(float);
extern void timer6_init(float);

volatile unsigned long int delay_counter = 0;

#endif