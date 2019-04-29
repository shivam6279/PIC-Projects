#ifndef _Xbee_ISR_H_
#define _Xbee_ISR_H_

#include "Xbee.h"

void __ISR_AT_VECTOR(_TIMER_7_VECTOR, IPL4SRS) general_purpose_1KHz(void) {
    IFS1bits.T7IF = 0;
    altitude_timer++;
    ToF_counter++;
    tx_buffer_timer++;
    if(safety_counter < 500) {
        safety_counter++;
    } 
    else if(safety_counter == 500) {
        XBeeReset();
        safety_counter = 501;
    }
}

#endif

