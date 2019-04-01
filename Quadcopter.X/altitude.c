#include "altitude.h"
#include "10DOF.h"
#include "PID.h"
#include "AHRS.h"
#include "settings.h"

void LoopAltitude(unsigned char *altitude_stage, unsigned long int *raw_pressure, unsigned long int *raw_temperature, double altitude_buffer[], double take_off_altitude, double *temperature) {
    int i;
    #ifdef BMP180
        if(*altitude_stage == 0) {
            StartTemperatureRead();//Initiate temperature read
            altitude_timer = 0;
            *altitude_stage = 1;
        }
        else if(*altitude_stage == 1 && altitude_timer > 5) {
            ReadRawTemperature();//After 5ms read temperature
            *temperature = ComputeTemperature();
            StartPressureRead();
            altitude_timer = 0;
            *altitude_stage = 2;
        }
        else if(*altitude_stage == 2 && altitude_timer > oversampling_delay) {
            altitude_KF_update(GetAltitude());

            StartTemperatureRead();
            altitude_timer = 0;
            *altitude_stage = 1;
        }
    #endif
    #ifdef MS5611            
        if(*altitude_stage == 0) {
            StartPressureRead();
            altitude_timer = 0;
            *altitude_stage = 1;
        }            
        else if(*altitude_stage == 1 && altitude_timer > oversampling_delay) {
            *raw_pressure = ReadRawPressure();
            StartTemperatureRead();
            altitude_timer = 0;
            *altitude_stage = 2;
        }
        else if(*altitude_stage == 2 && altitude_timer > oversampling_delay) {
            *raw_temperature = ReadRawTemperature();
            *temperature = ComputeTemperature(*raw_temperature);
            
            altitude_KF_update(GetAltitude(ComputePressure(*raw_pressure, *raw_temperature)));

            StartPressureRead();
            altitude_timer = 0;
            *altitude_stage = 1;
        }
    #endif
}

double GetTakeoffAltitude(double* buffer) {
    int i;
    double r = 0.0;
    /*
    for(i = 0; i < 100; i++) {
        #ifdef BMP180
            StartTemperatureRead();
            delay_ms(5);
            ReadRawTemperature();
            StartPressureRead();
            delay_ms(oversampling_delay);
            r += GetAltitude();
        #endif
        #ifdef MS5611
            r += GetAltitude(GetPressure());
        #endif
    }
    */
    for(i = 0; i < ALTITUDE_BUFFER_SIZE; i++) {
        #ifdef BMP180
            StartTemperatureRead();
            delay_ms(5);
            ReadRawTemperature();
            StartPressureRead();
            delay_ms(oversampling_delay);
            buffer[i] = GetAltitude();
        #endif
        #ifdef MS5611
            buffer[i] = GetAltitude(GetPressure());
        #endif
        r += buffer[i];
        
        altitude_KF_update(buffer[i]);
    }
    return r / (double)ALTITUDE_BUFFER_SIZE;
}