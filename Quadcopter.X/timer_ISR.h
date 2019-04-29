#ifndef _timer_ISR_H_
#define _timer_ISR_H_

#include "PID.h"
#include "pic32.h"

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL7SRS) pid_loop_timer(void){
    IFS0bits.T4IF = 0;
    esc_counter++;
    data_aq_counter++;
}

void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL4SRS) pwm(void){
    IFS0bits.T3IF = 0;
}

#endif
