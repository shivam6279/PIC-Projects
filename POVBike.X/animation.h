#ifndef _animation_H_
#define _animation_H_

#include "LED.h"

#define size 144
#define size2 1000
#define scale 1.0

extern unsigned char ppm_temp[];

extern unsigned char ppm[];

extern void polar_image(struct led *, struct led [size][size], double);

#endif