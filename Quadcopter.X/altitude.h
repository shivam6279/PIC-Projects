#ifndef _altitude_H_
#define _altitude_H_

#define ALTITUDE_BUFFER_SIZE 3

extern bool LoopAltitude(float*, float*);
extern double GetTakeoffAltitude();

#if ALTITUDE_BUFFER_SIZE > 0
    extern float altitude_buffer[ALTITUDE_BUFFER_SIZE];
#endif
#endif