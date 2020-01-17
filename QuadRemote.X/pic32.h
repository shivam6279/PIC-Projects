#ifndef _pic32_H_
#define _pic32_H_

extern void adc_init();
extern void pwm_init(float);
extern void write_pwm(int, unsigned char);
extern void init();
extern void delay_ms(unsigned int);
extern void timer2_init(float);
extern void timer3_init(float);
extern void timer4_init(float);
extern void timer5_init(float);

extern unsigned long int delay_counter;

#endif