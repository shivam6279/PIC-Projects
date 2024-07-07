#ifndef _LIS3MDL_H_
#define _LIS3MDL_H_

#include "settings.h"
#include <stdbool.h>

#define LIS3MDL_ADDR 0x1C

extern void LIS3MDL_Init();
extern bool LIS3MDL_GetCompass(float*, float*, float*);

#endif
