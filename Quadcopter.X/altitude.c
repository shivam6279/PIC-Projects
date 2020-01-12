#include <stdbool.h>
#include "altitude.h"
#include "10DOF.h"
#include "PID.h"
#include "AHRS.h"
#include "settings.h"

#if ALTITUDE_BUFFER_SIZE > 0
    float altitude_buffer[ALTITUDE_BUFFER_SIZE];
#endif

bool LoopAltitude(float *altitude, float *temperature, bool restart) {
    int i;
    
    bool ret = false;
    
    static unsigned char altitude_stage = 0;
    static unsigned long int raw_pressure;
    static unsigned long int raw_temperature;
    //static unsigned char altitude_stage;
    
    #ifdef BMP180
        if(altitude_stage == 0) {
            StartTemperatureRead();//Initiate temperature read
            altitude_timer = 0;
            *altitude_stage = 1;
        }
        else if(altitude_stage == 1 && altitude_timer > 5) {
            ReadRawTemperature();//After 5ms read temperature
            *temperature = ComputeTemperature();
            StartPressureRead();
            altitude_timer = 0;
            *altitude_stage = 2;
        }
        else if(*altitude_stage == 2 && altitude_timer > oversampling_delay) {
            *altitude = GetAltitude();
            //altitude_KF_update(GetAltitude());
            ret = true;

            StartTemperatureRead();
            altitude_timer = 0;
            *altitude_stage = 1;
        }
    #endif
    #ifdef MS5611            
        if(altitude_stage == 0) {
            StartPressureRead();
            altitude_timer = 0;
            altitude_stage = 1;
        }            
        else if(altitude_stage == 1 && altitude_timer > oversampling_delay) {
            raw_pressure = ReadRawPressure();
            StartTemperatureRead();
            altitude_timer = 0;
            altitude_stage = 2;
        }
        else if(altitude_stage == 2 && altitude_timer > oversampling_delay) {
            raw_temperature = ReadRawTemperature();
            *temperature = ComputeTemperature(raw_temperature);
            
            *altitude = GetAltitude(ComputePressure(raw_pressure, raw_temperature));
            //altitude_KF_update(GetAltitude(ComputePressure(raw_pressure, raw_temperature)));
            
            ret = true;
            
            if(restart) {
                StartPressureRead();                
                altitude_stage = 1;
            } else {
                altitude_stage = 0;
            }
            altitude_timer = 0;
        }
    #endif
    
#if ALTITUDE_BUFFER_SIZE > 0
    if(ret) {
        for(i = (ALTITUDE_BUFFER_SIZE - 1); i >= 1; i--)
            altitude_buffer[i] = altitude_buffer[i - 1];

        altitude_buffer[0] = *altitude;
        
        for(i = 1; i < ALTITUDE_BUFFER_SIZE; i++)
            *altitude += altitude_buffer[i];
        
        *altitude /= (float)ALTITUDE_BUFFER_SIZE;
    }
#endif
    return ret;
}

double GetTakeoffAltitude() {
    int i;
    double r = 0.0;
    
    for(i = 0; i < 100 - ALTITUDE_BUFFER_SIZE; i++) {
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
    
    for(i = (ALTITUDE_BUFFER_SIZE - 1); i >= 0; i--) {
        #ifdef BMP180
            StartTemperatureRead();
            delay_ms(5);
            ReadRawTemperature();
            StartPressureRead();
            delay_ms(oversampling_delay);
            altitude_buffer[i] = GetAltitude();
        #endif
        #ifdef MS5611
            altitude_buffer[i] = GetAltitude(GetPressure());
        #endif
        r += altitude_buffer[i];
        
        altitude_KF_update(altitude_buffer[i]);
    }
    return r / 100.0f;
}