#ifndef _altitude_H_
#define _altitude_H_

#define ALTITUDE_BUFFER_SIZE 5

extern bool LoopAltitude(float*, float*);
extern double GetTakeoffAltitude();

extern float max_altitude_rate;

#if ALTITUDE_BUFFER_SIZE > 0
    extern float altitude_buffer[ALTITUDE_BUFFER_SIZE];
#endif

#endif