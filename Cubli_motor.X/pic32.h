#ifndef _pic32_H_
#define _pic32_H_

#include <xc.h>

#define LED LATAbits.LATA10

extern void PICInit();
extern void ChangeNotificationInit();
extern void QEI_init();

extern void StartDelaymsCounter();
extern void StopDelaymsCounter();
extern unsigned long int ms_counter();
extern unsigned long int ms_counter2();
extern unsigned long int ms_counter3();
extern void reset_ms_counter();
extern void reset_ms_counter2();
extern void reset_ms_counter3();
extern void delay_ms(unsigned int);

extern void StartDelayusCounter();
extern void StopDelayusCounter();
extern unsigned long int us_counter();
extern void reset_us_counter();
extern void delay_us(unsigned int);

extern void timer2_init(float);
extern void timer3_init(float);
extern void timer4_init(float);
extern void timer5_init(float);
extern void timer6_init(float);
extern void timer7_init(float);

#endif