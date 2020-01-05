#ifndef _timer_H_
#define _timer_H_

#include "pic32.h"

void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4SRS) delay_timer(void) {
    IFS0bits.T2IF = 0;
    delay_counter++;
}

#endif
