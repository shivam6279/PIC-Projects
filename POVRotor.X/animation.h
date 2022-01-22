#ifndef _animation_H_
#define _animation_H_

#include "LED.h"

#define SCALE_FACTOR 1
#define size 200

extern void polar_image(struct led *, struct led[size][size], double);

extern void scaleBrightness_image(struct led[size][size], float);

extern unsigned char gif_init();
extern unsigned char gif_get_frame(struct led image[size][size], unsigned char);

extern unsigned char ppm_temp[];

extern const unsigned char ppm[];
extern const unsigned char gif[];

#endif