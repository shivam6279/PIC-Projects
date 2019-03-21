#ifndef _Xbee_ISR_H_
#define _Xbee_ISR_H_

#include "Xbee.h"

void __ISR_AT_VECTOR(_UART1_RX_VECTOR, IPL6SRS) Xbee_rx(void) {
    IFS3bits.U1RXIF = 0; 
    do{
        XBee_rx_byte = U1RXREG & 0xFF;

        if(XBee_rx_byte >> 5 == 0) {
            Xbee.x1 = (XBee_rx_byte & 0x1F) - 15;
        }

        else if(XBee_rx_byte >> 5 == 1) {
            Xbee.y1 = (XBee_rx_byte & 0x1F) - 15;
        }

        else if(XBee_rx_byte >> 5 == 2) {
            Xbee.x2 = (XBee_rx_byte & 0x1F) - 15;
        }

        else if(XBee_rx_byte >> 5 == 3) {
            Xbee.y2 = (XBee_rx_byte & 0x1F);
        }

        else if(XBee_rx_byte >> 5 == 4) {
            Xbee.ls = (XBee_rx_byte >> 1) & 1; 
            Xbee.rs = XBee_rx_byte & 1;
        }

        else if(XBee_rx_byte >> 5 == 5) { 
            Xbee.d2 = (XBee_rx_byte & 0b00001100) << 2;
            Xbee.d1 = (XBee_rx_byte & 0b00000011);
            safety_counter = 0;
            Xbee.signal = 1;
        }
    }while(U1STAbits.URXDA);

    IFS3bits.U1RXIF = 0; 
}

void __ISR_AT_VECTOR(_TIMER_6_VECTOR, IPL4SRS) Xbee_tx(void) {
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

void __ISR_AT_VECTOR(_TIMER_7_VECTOR, IPL4SRS) safety_timer(void) {
    IFS1bits.T7IF = 0;
    altitude_timer++;

    if(safety_counter >= 500) { 
        Xbee.signal = 0; 
        Xbee.x1 = 0; 
        Xbee.y1 = 0; 
        Xbee.x2 = 0; 
        Xbee.y2 = 0; 
    }
    else safety_counter++;
}

#endif

