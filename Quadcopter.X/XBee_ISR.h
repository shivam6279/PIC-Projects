#ifndef _Xbee_ISR_H_
#define _Xbee_ISR_H_

#include "Xbee.h"

void __ISR_AT_VECTOR(_UART1_RX_VECTOR, IPL6SRS) Xbee_rx(void) {
    IFS3bits.U1RXIF = 0; 

    static unsigned char XBee_rx_byte, XBee_address;

    do {
        XBee_rx_byte = U1RXREG & 0xFF;
        XBee_address = XBee_rx_byte >> 5;

        switch(XBee_address) {
            case 0:
                XBee_temp.x1 = (XBee_rx_byte & 0x1F) - 15;
                XBee_signal_temp = 1;
                break;

            case 1:
                XBee_temp.y1 = (XBee_rx_byte & 0x1F) - 15;
                break;

            case 2:
                XBee_temp.x2 = (XBee_rx_byte & 0x1F) - 15;
                break;

            case 3:
                XBee_temp.y2 = (XBee_rx_byte & 0x1F);
                break;

            case 4:
                XBee_temp.ls = (XBee_rx_byte >> 1) & 1; 
                XBee_temp.rs = XBee_rx_byte & 1;
                break;

            case 5: 
                XBee_temp.d2 = (XBee_rx_byte & 0b00001100) << 2;
                XBee_temp.d1 = (XBee_rx_byte & 0b00000011);
                safety_counter = 0;
                if(XBee_signal_temp) {
                    XBee_temp.signal = 1;
                    XBee_temp.data_ready = 1;

                    XBee = XBee_temp;
                }
                break;
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
    ToF_counter++;
    //tx_buffer_timer++;
    if(safety_counter >= 500) {
        XBeeReset();
    }
    else safety_counter++;
}

#endif

