#ifndef _GPS_H_
#define _GPS_H_

#include <stdbool.h>

extern double DifferenceLatLon(double, double, double, double);
extern float DifferenceBearing(double, double, double, double);
extern void GetLocation();

extern volatile double latitude, longitude;
extern float GPS_altitude;
extern unsigned int GPS_counter;
extern unsigned char GPS_time_counter;
extern char lat_str[16], lon_str[16], alt_str[6];
extern unsigned char receive5, GPS_stage;
extern volatile bool GPS_signal, GPS_connected ;

#endif