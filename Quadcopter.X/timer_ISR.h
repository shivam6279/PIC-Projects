#ifndef _timer_ISR_H_
#define _timer_ISR_H_

#include "PID.h"
#include "pic32.h"

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL7SRS) pid_loop_timer(void){
    IFS0bits.T4IF = 0;
    loop_counter++;
}

void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4SRS) delay_timer(void){
    IFS0bits.T2IF = 0;
    delay_counter++;
}

#endif
