#ifndef _GPS_ISR_H_
#define _GPS_ISR_H_

#include "GPS.h"

void __ISR_AT_VECTOR(_UART5_RX_VECTOR, IPL6SRS) GPS_(void){
    IFS5bits.U5RXIF = 0; 
    TMR5 = 0;
    do{ receive5 = U5RXREG & 0xFF;
        if(receive5 == '$') GPS_stage = 1;
        else if(GPS_stage == 1) { 
            if(receive5 == 'G') GPS_stage++; 
            else GPS_stage = 0; 
        }
        else if(GPS_stage == 2) { 
            if(receive5 == 'P') GPS_stage++; 
            else GPS_stage = 0; 
        }
        else if(GPS_stage == 3) { 
            if(receive5 == 'G') GPS_stage++;
            else GPS_stage = 0; 
        }
        else if(GPS_stage == 4) { 
            if(receive5 == 'G') GPS_stage++; 
            else GPS_stage = 0; 
        }
        else if(GPS_stage == 5) { 
            if(receive5 == 'A') GPS_stage++; 
            else GPS_stage = 0; 
        }
        else if(GPS_stage == 6 && receive5 == ',') { 
            GPS_connected = 1;
            GPS_time_counter = 0; 
            GPS_stage++; 
        }  
        else if(GPS_stage == 7 && receive5 == ',') { 
            GPS_stage++; 
            GPS_counter = 0; 
        }
        else if(GPS_stage == 8) { 
            if(receive5 == 'N' || receive5 == 'S') { 
                GPS_stage++; lat_str[GPS_counter++] = receive5; 
                lat_str[GPS_counter] = '\0';
            } 
            else lat_str[GPS_counter++] = receive5; 
        }
        else if(GPS_stage == 9 && receive5 == ',') { 
            GPS_stage++; GPS_counter = 0; 
        }    
        else if(GPS_stage == 10) { 
            if(receive5 == 'E' || receive5 == 'W') { 
                GPS_stage++; lon_str[GPS_counter++] = receive5; 
                lon_str[GPS_counter] = '\0'; 
            } 
            else lon_str[GPS_counter++] = receive5; 
        }
        else if(GPS_stage == 11 && receive5 == ',') {
            GPS_stage++;
        }
        else if(GPS_stage == 12) { 
            if(receive5 == '0') { 
                GPS_signal = 0; 
                GPS_stage = 0; 
            } else { 
                GPS_stage++; 
                GPS_counter = 0; 
                GPS_signal = 1; 
            }
        }
        else if(GPS_stage == 13 && receive5 == ',') { 
            GPS_stage++; 
        }
        else if(GPS_stage == 14 && receive5 == ',') { 
            GPS_stage++; 
        }
        else if(GPS_stage == 15 && receive5 == ',') { 
            GPS_stage++; 
            GPS_counter = 0; 
        }
        else if(GPS_stage == 16) { 
            if(receive5 == ',') { 
                GPS_stage = 0; alt_str[GPS_counter] = '\0'; 
            } 
            else alt_str[GPS_counter++] = receive5; 
            GetLocation(); 
        }
    }while(U5STAbits.URXDA); 
    IFS5bits.U5RXIF = 0; 
    
    IFS5bits.U5RXIF = 0; 
}

void __ISR_AT_VECTOR(_TIMER_5_VECTOR, IPL4SRS) GPS_timer(void){
    IFS0bits.T5IF = 0;
    if(GPS_time_counter == 20) {
        GPS_signal = 0;
        GPS_connected = 0; 
    }
    else GPS_time_counter++;    
}

#endif