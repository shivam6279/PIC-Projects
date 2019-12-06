#ifndef _animation_H_
#define _animation_H_

#include "LED.h"

#define size 144
#define size2 1000
#define scale 1.0

extern void polar_image(struct led *, struct led [size][size], double);
extern void polar_image_test(struct led *, struct led [size2][size2], double);

extern unsigned char ppm_temp[];

extern unsigned char ppm[];

#endif