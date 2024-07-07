#ifndef _GPS_H_
#define _GPS_H_

#include <stdbool.h>

extern unsigned char parse_nmea();
extern double DifferenceLatLon(double, double, double, double);
extern float DifferenceBearing(double, double, double, double);

volatile extern double latitude, longitude;
volatile extern float gps_altitude;
extern volatile bool gps_signal, gps_connected;
extern volatile char nmea_str[84];

#endif