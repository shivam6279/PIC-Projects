#ifndef _GPS_H_
#define _GPS_H_

#include <stdbool.h>

extern double DifferenceLatLon(double, double, double, double);
extern float DifferenceBearing(double, double, double, double);
extern void GetLocation();

volatile double latitude = 0.0, longitude = 0.0;
float GPS_altitude = 0.0;
unsigned int GPS_counter = 0;
unsigned char GPS_time_counter = 0;
char lat_str[16] = {'V', '\0'}, lon_str[16] = {'V', '\0'}, alt_str[6] = {'V', '\0'};
unsigned char receive5, GPS_stage = 0;
volatile bool GPS_signal = 0, GPS_connected = 0;

#endif