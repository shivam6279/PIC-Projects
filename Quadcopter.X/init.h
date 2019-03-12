#ifndef _init_H_
#define _init_H_

#include "PID.h"

extern void Init();
extern void ResetPID(PID*, PID*, PID*, PID*, PID*);
extern void ResetQuaternion(float[]);

#endif
