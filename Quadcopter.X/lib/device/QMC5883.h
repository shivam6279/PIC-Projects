#ifndef _QMC5883_H_
#define _QMC5883_H_

#include "settings.h"
#include <stdbool.h>

#define QMC5883_ADDR 0x0D

extern void QMC5883_Init();
extern bool QMC5883_GetCompass(float*, float*, float*);

#endif
