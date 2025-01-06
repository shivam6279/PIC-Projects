#ifndef _animation_H_
#define _animation_H_

#include "LED.h"

#define SCALE_FACTOR 1.0
#define size 275
#define size_2 137.5 // size / 2

extern void polar_image(led *, led[size][size], double);

extern void scaleBrightness_image(led[size][size], float);

extern unsigned char gif_init();
extern unsigned char gif_get_frame(led image[size][size], unsigned char);

extern unsigned char ppm_temp[];

extern const unsigned char ppm[];
extern const unsigned char gif[];

#endif