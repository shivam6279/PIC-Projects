#ifndef _GPS_ISR_H_
#define _GPS_ISR_H_

#include "GPS.h"

void __ISR_AT_VECTOR(_UART5_RX_VECTOR, IPL6SRS) GPS_(void){
    IFS5bits.U5RXIF = 0; 
    TMR5 = 0;
    do{ GPS_rx_byte = U5RXREG & 0xFF;
        if(GPS_rx_byte == '$') GPS_stage = 1;
        else if(GPS_stage == 1) { 
            if(GPS_rx_byte == 'G') GPS_stage++; 
            else GPS_stage = 0; 
        }
        else if(GPS_stage == 2) { 
            if(GPS_rx_byte == 'P') GPS_stage++; 
            else GPS_stage = 0; 
        }
        else if(GPS_stage == 3) { 
            if(GPS_rx_byte == 'G') GPS_stage++;
            else GPS_stage = 0; 
        }
        else if(GPS_stage == 4) { 
            if(GPS_rx_byte == 'G') GPS_stage++; 
            else GPS_stage = 0; 
        }
        else if(GPS_stage == 5) { 
            if(GPS_rx_byte == 'A') GPS_stage++; 
            else GPS_stage = 0; 
        }
        else if(GPS_stage == 6 && GPS_rx_byte == ',') { 
            GPS_connected = 1;
            GPS_time_counter = 0; 
            GPS_stage++; 
        }  
        else if(GPS_stage == 7 && GPS_rx_byte == ',') { 
            GPS_stage++; 
            GPS_counter = 0; 
        }
        else if(GPS_stage == 8) { 
            if(GPS_rx_byte == 'N' || GPS_rx_byte == 'S') { 
                GPS_stage++; lat_str[GPS_counter++] = GPS_rx_byte; 
                lat_str[GPS_counter] = '\0';
            } 
            else lat_str[GPS_counter++] = GPS_rx_byte; 
        }
        else if(GPS_stage == 9 && GPS_rx_byte == ',') { 
            GPS_stage++; GPS_counter = 0; 
        }    
        else if(GPS_stage == 10) { 
            if(GPS_rx_byte == 'E' || GPS_rx_byte == 'W') { 
                GPS_stage++; lon_str[GPS_counter++] = GPS_rx_byte; 
                lon_str[GPS_counter] = '\0'; 
            } 
            else lon_str[GPS_counter++] = GPS_rx_byte; 
        }
        else if(GPS_stage == 11 && GPS_rx_byte == ',') {
            GPS_stage++;
        }
        else if(GPS_stage == 12) { 
            if(GPS_rx_byte == '0') { 
                GPS_signal = 0; 
                GPS_stage = 0; 
            } else { 
                GPS_stage++; 
                GPS_counter = 0; 
                GPS_signal = 1; 
            }
        }
        else if(GPS_stage == 13 && GPS_rx_byte == ',') { 
            GPS_stage++; 
        }
        else if(GPS_stage == 14 && GPS_rx_byte == ',') { 
            GPS_stage++; 
        }
        else if(GPS_stage == 15 && GPS_rx_byte == ',') { 
            GPS_stage++; 
            GPS_counter = 0; 
        }
        else if(GPS_stage == 16) { 
            if(GPS_rx_byte == ',') { 
                GPS_stage = 0; alt_str[GPS_counter] = '\0'; 
            } 
            else alt_str[GPS_counter++] = GPS_rx_byte; 
            GetLocation(); 
        }
    }while(U5STAbits.URXDA); 
    IFS5bits.U5RXIF = 0; 
    
    IFS5bits.U5RXIF = 0; 
}

#ifndef board_v4
void __ISR_AT_VECTOR(_TIMER_5_VECTOR, IPL4SRS) GPS_timer(void) {
#else
void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL4SRS) GPS_timer(void) { 
#endif
        IFS0bits.T5IF = 0;
        if(GPS_time_counter == 20) {
            GPS_signal = 0;
            GPS_connected = 0; 
        }
        else GPS_time_counter++;    
    }

#endif