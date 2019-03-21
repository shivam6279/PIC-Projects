#include "GPS.h"
#include "settings.h"
#include <math.h>

volatile double latitude = 0.0, longitude = 0.0;
float GPS_altitude = 0.0;
unsigned int GPS_counter = 0;
unsigned char GPS_time_counter = 0;
char lat_str[16] = {'V', '\0'}, lon_str[16] = {'V', '\0'}, alt_str[6] = {'V', '\0'};
unsigned char GPS_rx_byte, GPS_stage = 0;
volatile bool GPS_signal = 0, GPS_connected = 0;

double DifferenceLatLon(double lat1, double lon1, double lat2, double lon2){
    double a;
    lat1 /= RAD_TO_DEGREES; 
    lat2 /= RAD_TO_DEGREES; 
    lon1 /= RAD_TO_DEGREES; 
    lon2 /= RAD_TO_DEGREES;
    a = pow(sin((lat2 - lat1) / 2), 2) + cos(lat1) * cos(lat2) * pow(sin((lon2 - lon1) / 2), 2);
    a = 2 * atan2(sqrt(a), sqrt(1 - a));
    return (6371000.0 * a);
}

float DifferenceBearing(double lat1, double lon1, double lat2, double lon2){
    double r;
    lat1 /= RAD_TO_DEGREES; 
    lat2 /= RAD_TO_DEGREES; 
    lon1 /= RAD_TO_DEGREES; 
    lon2 /= RAD_TO_DEGREES;
    r = (atan2(-(sin(lon2 - lon1) * cos(lat2)), (cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1)))) * RAD_TO_DEGREES;
    return r;
}

void GetLocation(){
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