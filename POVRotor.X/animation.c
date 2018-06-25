#include "LED.h"
#include "draw.h"
#include <math.h>

#define size 144
#define scale 1.0
void polar_image(struct led *buffer, struct led image[size][size], double angle){
    int r;
    int x, y;
    limit_angle(&angle);
    for(r = 1; r < LED_LENGTH; r+=2) {
        x = scale * ((double)r * cos(angle * 3.1415 / 180.0) + 0.5) + size / 2;
        y = scale * ((double)r * sin(angle * 3.1415 / 180.0) + 0.5) + size / 2;
        if((x >= size || x < 0) || (y >= size || y < 0)) break;
        buffer[LED_LENGTH / 2 - (r+1)/2] = image[y][x];
    }
    angle = angle + OPP_ANGLE_CORRECTION + 180.0;
    limit_angle(&angle);
    for(r = 2; r < LED_LENGTH; r+=2) {
        x = scale * ((double)r * cos(angle * 3.1415 / 180.0) + 0.5) + size / 2;
        y = scale * ((double)r * sin(angle * 3.1415 / 180.0) + 0.5) + size / 2;
        if((x >= size || x < 0) || (y >= size || y < 0)) break;
        buffer[LED_LENGTH / 2 + r/2] = image[y][x];
    }
}