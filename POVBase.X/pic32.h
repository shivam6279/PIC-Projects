#ifndef _pic32_H_
#define _pic32_H_

#include <xc.h>

#define MOTOR_LED LATBbits.LATB0
#define COIL_LED LATBbits.LATB1

#define BUTTON PORTGbits.RG6

volatile unsigned char mode;

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

extern void timer2_init(float);
extern void timer3_init(float);
extern void timer4_init(float);
extern void timer5_init(float);
extern void timer6_init(float);
extern void timer7_init(float);

#endif