#ifndef _init_H_
#define _init_H_

#include <stdbool.h>

#include "PID.h"

extern void Init();
extern void ResetQuaternion(float[]);
extern void I2C_device_list(bool addr_list[128]);

#endif
