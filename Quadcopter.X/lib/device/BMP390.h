#ifndef _BMP390_H_
#define _BMP390_H_

#include <stdbool.h>

//BMP390
#define BMP390_ADDR 0x76

#define SEA_LEVEL_PRESSURE 101730.0f

extern void BMP390_read_calibration_data();
extern void BMP390_Init();
extern unsigned long BMP390_read_pressure();
extern unsigned long BMP390_read_temp();
extern void BMP390_read_pressure_temp(unsigned long*, unsigned long*);
extern float BMP390_compensate_temp(float);
extern float BMP390_compensate_pressure(float, float);

#endif
