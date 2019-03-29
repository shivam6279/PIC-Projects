#include "altitude.h"
#include "10DOF.h"
#include "PID.h"
#include "settings.h"

void LoopAltitude(unsigned char *altitude_stage, unsigned long int *raw_pressure, unsigned long int *raw_temperature, double altitude_buffer[], PID *altitude, double take_off_altitude, double *temperature) {
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
            for(i = (ALTITUDE_BUFFER_SIZE - 1); i >= 1; i--) altitude_buffer[i] = altitude_buffer[i - 1];
            altitude_buffer[0] = GetAltitude();
            for(i = 0, altitude->error = 0.0; i < ALTITUDE_BUFFER_SIZE; i++) altitude->error += altitude_buffer[i];
            altitude->error = (altitude->error / (double)ALTITUDE_BUFFER_SIZE) - take_off_altitude; 

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
            for(i = (ALTITUDE_BUFFER_SIZE - 1); i >= 1; i--) 
                altitude_buffer[i] = altitude_buffer[i - 1];
            altitude_buffer[0] = GetAltitude(ComputePressure(*raw_pressure, *raw_temperature));
            for(i = 0, altitude->error = 0.0; i < ALTITUDE_BUFFER_SIZE; i++) 
                altitude->error += altitude_buffer[i];
            altitude->error = (altitude->error / (double)ALTITUDE_BUFFER_SIZE) - take_off_altitude; 

            StartPressureRead();
            altitude_timer = 0;
            *altitude_stage = 1;
        }
    #endif
}

double GetTakeoffAltitude(double* buffer) {
    int i;
    double r;
    for(i = 0, r = 0.0; i < 100; i++) {
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
    }
    return r / (100.0 + (double)ALTITUDE_BUFFER_SIZE);
}