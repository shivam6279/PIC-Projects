#ifndef _pic32_H_
#define _pic32_H_

#define LOOP_TIMER_ON T4CONbits.ON 
#define TX_TIMER_ON T6CONbits.ON

extern void PICInit();
extern void delay_ms(unsigned int);
extern void timer2_init(unsigned long int);
extern void timer3_init(unsigned long int);
extern void timer4_init(unsigned long int);
extern void timer5_init(unsigned long int);
extern void timer6_init(unsigned long int);

unsigned long int delay_counter = 0;

#endif