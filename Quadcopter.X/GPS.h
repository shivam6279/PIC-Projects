#ifndef _GPS_H_
#define _GPS_H_

#include <stdbool.h>

extern double DifferenceLatLon(double, double, double, double);
extern float DifferenceBearing(double, double, double, double);
extern void GetLocation();

volatile extern double latitude, longitude;
volatile extern float GPS_altitude;
extern volatile bool GPS_signal, GPS_connected;

#endif