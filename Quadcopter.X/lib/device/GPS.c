#include "GPS.h"
#include "settings.h"
#include <math.h>
#include <xc.h>
#include <string.h>
#include <stdlib.h>
#include "USART.h"
#include <sys/attribs.h>

volatile double latitude = 0.0, longitude = 0.0;
volatile float gps_altitude = 0.0;

static volatile unsigned int gps_counter = 0;
static volatile unsigned char gps_time_counter = 0;
volatile char nmea_str[84] = {'\0'};
static volatile unsigned char nmea_index = 0;
volatile bool gps_signal = 0, gps_connected = 0;

void __ISR_AT_VECTOR(_UART6_RX_VECTOR, IPL6SRS) GPS_(void){
    IFS5bits.U6RXIF = 0; 
    TMR5 = 0;
    static uint8_t i;

    static unsigned char gps_rx_byte, gps_stage = 0;
    
    do{ 
        gps_rx_byte = U6RXREG & 0xFF;
        
        if(nmea_index < 82) {
            nmea_str[nmea_index++] = gps_rx_byte;
        }
        if(gps_rx_byte == '\n') {
            nmea_str[nmea_index] = '\0';
            for(i = 0; nmea_str[i] != '\0' && i < 84; i++) {
                nmea_str[i] = nmea_str[i];
            }
            parse_nmea();
            nmea_index = 0;
        }
    }while(U6STAbits.URXDA); 
    IFS5bits.U6RXIF = 0; 
    
    IFS5bits.U6RXIF = 0; 
}

void __ISR_AT_VECTOR(_TIMER_5_VECTOR, IPL4SRS) GPS_timer(void) {
    IFS0bits.T5IF = 0;
    if(gps_time_counter == 12) {
        gps_signal = 0;
        gps_connected = 0; 
        nmea_str[0] = '\0';
    } else {
        gps_time_counter++;
    }
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

unsigned char parse_nmea() {
//    $GNGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,M,<10>,M,< 11>,<12>*xx<CR><LF>
//    <0> $GNGGA
//    <1> UTC time, the format is hhmmss.sss
//    <2> Latitude, the format is ddmm.mmmmmmm
//    <3> Latitude hemisphere, N or S (north latitude or south latitude)
//    <4> Longitude, the format is dddmm.mmmmmmm
//    <5> Longitude hemisphere, E or W (east longitude or west longitude)
//    <6> GNSS positioning status: 0 not positioned, 1 single point positioning, 2 differential GPS fixed solution, 4 fixed solution, 5 floating point solution
//    <7> Number of satellites used
//    <8> HDOP level precision factor
//    <9> Altitude
//    <10> The height of the earth ellipsoid relative to the geoid
//    <11> Differential time
//    <12> Differential reference base station label
//    * Statement end marker
//    xx XOR check value of all bytes starting from $ to *
//    <CR> Carriage return, end tag
//    <LF> line feed, end tag
    
    unsigned char i, j, k = 0;
    
    unsigned char str_len;
    static char deliminated[15][13];
    unsigned char nmea_copy[84];
    
    for(i = 0; nmea_str[i] != '\0' && i < 84; i++) {
        nmea_copy[i] = nmea_str[i];
    }
    nmea_copy[i] = '\0';
    str_len = i;
    
    if(strncmp("$GNGGA", nmea_copy, 6) != 0 && strncmp("$GPGGA", nmea_copy, 6) != 0) {
        return 0;
    }
    
    gps_connected = 1;
    gps_time_counter = 0;
    
    for(i = 0, k = 0;; i++) {
        for(j = 0; nmea_copy[k] != ',' && nmea_copy[k] != '\r'; j++) {
            deliminated[i][j] = nmea_copy[k++];
        }
        deliminated[i][j] = '\0';
        k++;
        if(nmea_copy[k] == '\r' || nmea_copy[k] == '\n') {
            break;
        }
    }
    
    if(deliminated[6][0] == '1' || deliminated[6][0] == '2' || deliminated[6][0] == '4' || deliminated[6][0] == '5') {
        gps_signal = 1;
    } else {
        gps_signal = 0;
        latitude = 0;
        longitude = 0;
        return 0;
    }
    
    unsigned char hour, minute, second;
    if(strlen(deliminated[1]) >= 6) {
        hour = (deliminated[1][0] - '0') * 10 + (deliminated[1][1] - '0');
        minute = (deliminated[1][2] - '0') * 10 + (deliminated[1][3] - '0');
        second = (deliminated[1][4] - '0') * 10 + (deliminated[1][5] - '0');
    }
    
    double latlon_temp;
    if(strlen(deliminated[2]) >= 10) {
        latitude = (float)(deliminated[2][0] - '0') * 10 + (float)(deliminated[2][1] - '0');
        latlon_temp = (float)(deliminated[2][2] - '0') * 10 + (float)(deliminated[2][3] - '0');
        latlon_temp += (float)(deliminated[2][5] - '0') / 10 + (float)(deliminated[2][6] - '0') / 100 + (float)(deliminated[2][7] - '0') / 1000 + (float)(deliminated[2][8] - '0') / 10000;
        latlon_temp += (float)(deliminated[2][9] - '0') / 100000 + (float)(deliminated[2][10] - '0') / 100000 + (float)(deliminated[2][11] - '0') / 10000000;
        latitude += latlon_temp /60.0f;
    }
    if(deliminated[3][0] == 'S') {
        latitude *= -1.0f;
    }
    
    if(strlen(deliminated[4]) >= 10) {
        longitude = (float)(deliminated[4][0] - '0') * 100 + (float)(deliminated[4][1] - '0') * 10 + (float)(deliminated[4][2] - '0');
        latlon_temp = (float)(deliminated[4][3] - '0') * 10 + (float)(deliminated[4][4] - '0');
        latlon_temp += (float)(deliminated[4][6] - '0') / 10 + (float)(deliminated[4][7] - '0') / 100 + (float)(deliminated[4][8] - '0') / 1000 + (float)(deliminated[4][9] - '0') / 10000;
        latlon_temp += (float)(deliminated[4][10] - '0') / 100000 + (float)(deliminated[4][11] - '0') / 100000 + (float)(deliminated[4][12] - '0') / 10000000;
        longitude += latlon_temp /60.0f;
    }
    if(deliminated[5][0] == 'W') {
        longitude *= -1.0f;
    }
    
    double altitude = atof(deliminated[9]);
    
    return 1;
}

/*float StrToFloat(char str[]) {
    static signed char c, left, right;
    static float ret, tens;
    
    if(str[0] == '-' || str[0] == '+') {
        c = 1;
    } else {
        c = 0;
    }
    
    for(left = 0; str[left] != '.' && str[left] != '\0'; left++);
    
    right = left + 1;
    left--;
    ret = 0.0;
        
    if(str[left + c + 1] == '\0') {
        for(tens = 1.0; left >= c; left--, tens *= 10.0)
            ret += (float)(str[left] - 48) * tens;
    } else {
        for(tens = 1.0; left >= c; left--, tens *= 10.0)
            ret += (float)(str[left] - 48) * tens;

        for(tens = 10.0; str[right] != '\0'; right++, tens *= 10.0)
            ret += (float)(str[right] - 48) / tens;
    }
    
    if(str[0] == '-')
        return -ret;
    
    return ret;
}*/