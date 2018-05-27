#ifndef _Xbee_ISR_H_
#define _Xbee_ISR_H_

#include "Xbee.h"

void __ISR_AT_VECTOR(_UART1_RX_VECTOR, IPL6SRS) Xbee_rx(void){
    IFS3bits.U1RXIF = 0; 
    do{
        receive1 = U1RXREG & 0xFF;
        if(receive1 >> 5 == 0) remote_x1 = (receive1 & 0x1F) - 15;
        else if(receive1 >> 5 == 1) {
            remote_y1 = (receive1 & 0x1F) - 15;
        }
        else if(receive1 >> 5 == 2) {
            remote_x2 = (receive1 & 0x1F) - 15;
        }
        else if(receive1 >> 5 == 3) {
            remote_y2 = (receive1 & 0x1F);
        }
        else if(receive1 >> 5 == 4) { 
            left_switch = (receive1 >> 1) & 1; 
            right_switch = receive1 & 1; 
        }
        else if(receive1 >> 5 == 5) { 
            dial2 = (receive1 & 0b00001100) << 2;
            dial1 = (receive1 & 0b00000011);
            safety_counter = 0;
            Xbee_signal = 1;
        }
    }while(U1STAbits.URXDA);
    IFS3bits.U1RXIF = 0; 
}

void __ISR_AT_VECTOR(_TIMER_6_VECTOR, IPL4SRS) Xbee_tx(void){
    IFS0bits.T6IF = 0;
    if(tx_flag) {
        while(!U1STAbits.UTXBF) {
            U1TXREG = tx_buffer[tx_buffer_index];
            if(tx_buffer[tx_buffer_index++] == '\r') {
                tx_buffer_index = 0;
                tx_flag = 0;
                break;
            }
        }
    }
}

void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL4SRS) safety_timer(void){
    IFS0bits.T3IF = 0;
    altitude_timer++;
    if(safety_counter == 500) { 
        Xbee_signal = 0; 
        remote_x1 = 0; 
        remote_y1 = 0; 
        remote_x2 = 0; 
        remote_y2 = 0; 
    }
    else safety_counter++;
}

#endif

