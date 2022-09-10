#include "GPS.h"
#include "settings.h"
#include <math.h>
#include <xc.h>
#include <sys/attribs.h>

volatile double latitude = 0.0, longitude = 0.0;
volatile float GPS_altitude = 0.0;

static unsigned int GPS_counter = 0;
static unsigned char GPS_time_counter = 0;
static char lat_str[16] = {'V', '\0'}, lon_str[16] = {'V', '\0'}, alt_str[6] = {'V', '\0'};

volatile bool GPS_signal = 0, GPS_connected = 0;

void __ISR_AT_VECTOR(_UART5_RX_VECTOR, IPL6SRS) GPS_(void){
    IFS5bits.U5RXIF = 0; 
    TMR5 = 0;

    static unsigned char GPS_rx_byte, GPS_stage = 0;

    do{ 
        GPS_rx_byte = U5RXREG & 0xFF;
        if(GPS_rx_byte == '$') GPS_stage = 1;
        else if(GPS_stage == 1) { 
            if(GPS_rx_byte == 'G') GPS_stage++; 
            else GPS_stage = 0; 
        }
        else if(GPS_stage == 2) { 
            if(GPS_rx_byte == 'P' || GPS_rx_byte == 'N') GPS_stage++; 
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

void __ISR_AT_VECTOR(_TIMER_5_VECTOR, IPL4SRS) GPS_timer(void) {
    IFS0bits.T5IF = 0;
    if(GPS_time_counter == 20) {
        GPS_signal = 0;
        GPS_connected = 0; 
    }
    else GPS_time_counter++;    
}

double DifferenceLatLon(double lat1, double lon1, double lat2, double lon2) {
    double a;
    lat1 = TO_RAD(lat1); 
    lat2 = TO_RAD(lat2); 
    lon1 = TO_RAD(lon1);  
    lon2 = TO_RAD(lon2); 
    a = pow(sin((lat2 - lat1) / 2), 2) + cos(lat1) * cos(lat2) * pow(sin((lon2 - lon1) / 2), 2);
    a = 2 * atan2(sqrt(a), sqrt(1 - a));
    return (6371000.0 * a);
}

float DifferenceBearing(double lat1, double lon1, double lat2, double lon2) {
    lat1 = TO_RAD(lat1); 
    lat2 = TO_RAD(lat2); 
    lon1 = TO_RAD(lon1);  
    lon2 = TO_RAD(lon2); 
    return TO_DEG(atan2(-(sin(lon2 - lon1) * cos(lat2)), (cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1))));
}

void GetLocation() {
    int left, i;
    double temp_left, temp_right, tens;
    if(!GPS_signal) { 
        latitude = 0.0; 
        longitude = 0.0; 
        return; 
    }
    for(i = 0; lat_str[i] != '.'; i++); 
    left = i - 2;
    for(i = left - 1, tens = 1.0, temp_left = 0.0; i >= 0; i--) {
        temp_left += (double)(lat_str[i] - 48) * tens;
        tens *= 10.0;
    }
    for(i = left, tens = 10.0, temp_right = 0.0; lat_str[i] != 'N' && lat_str[i] != 'S'; i++) { 
        if(lat_str[i] != '.') { 
            temp_right += (double)(lat_str[i] - 48) * tens; 
            tens /= 10;
        }
    }
    latitude = temp_left + (temp_right / 60.0);
    if(lat_str[i] == 'S') latitude *= (-1.0);
    
    for(i = 0; lon_str[i] != '.'; i++); 
    left = i - 2;
    for(i = left - 1, tens = 1.0, temp_left = 0.0; i >= 0; i--) {
        temp_left += (double)(lon_str[i] - 48) * tens;
        tens *= 10.0;
    }
    for(i = left, tens = 10.0, temp_right = 0.0; lon_str[i] != 'E' && lon_str[i] != 'W'; i++) { 
        if(lon_str[i] != '.') { 
            temp_right += (double)(lon_str[i] - 48) * tens; 
            tens /= 10; 
        }
    }
    longitude = temp_left + (temp_right / 60.0);
    if(lon_str[i] == 'W') longitude *= (-1.0);
    
    for(i = 0; alt_str[i] != '.'; i++); 
    left = i;
    for(i = left - 1, tens = 1.0, temp_left = 0.0; i >= 0; i--){
        temp_left += (float)(alt_str[i] - 48) * tens;
        tens *= 10.0;
    }
    for(i = left + 1, tens = 0.1, temp_right = 0.0; alt_str[i] != '\0'; i++) {
        temp_right += (float)(alt_str[i] - 48) / tens;
        tens /= 10.0;
    }
    GPS_altitude = temp_left + temp_right;
}