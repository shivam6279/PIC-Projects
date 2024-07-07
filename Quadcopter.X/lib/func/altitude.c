#include <stdbool.h>
#include "altitude.h"
#include "10DOF.h"
#include "pic32.h"
#include "PID.h"
#include "AHRS.h"
#include "settings.h"

#if ALTITUDE_BUFFER_SIZE > 0
    float altitude_buffer[ALTITUDE_BUFFER_SIZE];
#endif

double GetTakeoffAltitude() {
    int i;
    double r = 0.0;
    
#if ALTITUDE_BUFFER_SIZE > 0
    for(i = 0; i < 100 - ALTITUDE_BUFFER_SIZE; i++) {
        #ifdef BMP180
            StartTemperatureRead();
            delay_ms(5);
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
#else
    for(i = 0; i < 10; i++) {
        r += GetAltitude();
        delay_ms(50);
    }
    
    r /= 10.0f;
        
    altitude_KF_update(r);
    altitude_KF_update(r);
    altitude_KF_update(r);
    altitude_KF_update(r);
    altitude_KF_update(r);
#endif
    return r;
}