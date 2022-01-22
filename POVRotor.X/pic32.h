#ifndef _pic32_H_
#define _pic32_H_

#include <xc.h>

#define DELAY_TIMER_ON T2CONbits.ON
#define RPM_TIMER_ON T3CONbits.ON 
#define LED_TIMER_ON T4CONbits.ON 
#define uS_TIMER_ON T5CONbits.ON 

extern void PICInit();

extern void StartDelayCounter();
extern void StopDelayCounter();
extern unsigned long int ms_counter();
extern unsigned long int ms_counter2();
extern unsigned long int ms_counter3();
extern void delay_ms(unsigned int);

extern void set_ms_counter(unsigned long int);
extern void set_ms_counter2(unsigned long int);
extern void set_ms_counter3(unsigned long int);

extern void timer2_init(float);
extern void timer3_init(float);
extern void timer4_init(float);
extern void timer5_init(float);
extern void timer6_init(float);
extern void timer7_init(float);

extern volatile unsigned long us_counter, temp_us;

#endif