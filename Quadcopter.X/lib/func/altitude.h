#ifndef _altitude_H_
#define _altitude_H_

#include <stdbool.h>

#define ALTITUDE_BUFFER_SIZE 0

extern double GetTakeoffAltitude();

extern float max_altitude_rate;

#if ALTITUDE_BUFFER_SIZE > 0
    extern float altitude_buffer[ALTITUDE_BUFFER_SIZE];
#endif

#endif