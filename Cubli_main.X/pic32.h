#ifndef _pic32_H_
#define _pic32_H_

#include <xc.h>

#define DELAY_TIMER_ON T2CONbits.ON
#define SAFETY_TIMER_ON T7CONbits.ON
#define LOOP_TIMER_ON T4CONbits.ON 
#define GPS_TIMER_ON T5CONbits.ON

#define LED0 LATCbits.LATC14
#define LED1 LATBbits.LATB11

#define LED_ESC_A LATDbits.LATD4
#define LED_ESC_B LATFbits.LATF0
#define LED_ESC_C LATEbits.LATE0

#define LDO_XBEE LATFbits.LATF3
#define LDO_ESP LATBbits.LATB2

extern void PICInit();

extern void StartDelayCounter();
extern void StopDelayCounter();
extern unsigned long int ms_counter();
extern void delay_ms(unsigned int);

extern void timer2_init(float);
extern void timer3_init(float);
extern void timer4_init(float);
extern void timer5_init(float);
extern void timer6_init(float);
extern void timer7_init(float);

#endif