#include "LED.h"
#include "draw.h"
#include <math.h>

void pie(struct led buffer[2][LED_LENGTH + RADIUS_OFFSET], struct led *pie_colors, int n, double angle) {
    int i;
    limit_angle(&angle);
    int idx1 = angle / (360.0 / (double)n);
    
    for (i = 0; i < (LED_LENGTH + RADIUS_OFFSET); i++) {
        buffer[0][i] = pie_colors[idx1];
    }
    
    angle = angle + OPP_ANGLE_CORRECTION + 180.0;
    limit_angle(&angle);
    int idx2 = angle / (360.0 / (double)n);
    
    for (i = 0; i < (LED_LENGTH + RADIUS_OFFSET); i++) {
        buffer[1][i] = pie_colors[idx2];
    }
}

void polar(struct led *buffer, int (*f)(double), struct led color, double angle) {
    limit_angle(&angle);
    int r1 = f(angle);
    
    angle = angle + OPP_ANGLE_CORRECTION + 180.0;
    limit_angle(&angle);
    int r2 = f(angle);
    
    int i;
    for (i = 0; i < LED_LENGTH; i++) {
        buffer[i] = color_black;
    }
    
    if (r2 % 2 == 1) {
        buffer[LED_LENGTH / 2 + r2 / 2] = color;
    }
    if (r1 % 2 == 0) {
        buffer[LED_LENGTH / 2 - r1 / 2] = color;
    }
}

void polar_fill(struct led *buffer, int (*f)(double), struct led color, double angle) {
    limit_angle(&angle);
    int r1 = f(angle);
    
    angle = angle + OPP_ANGLE_CORRECTION + 180.0;
    limit_angle(&angle);
    int r2 = f(angle);
    
    int i;
    for (i = 0; i < LED_LENGTH; i++) {
        buffer[i] = color_black;
    }
    
    if (r2 % 2 == 1) {
        for(i = LED_LENGTH / 2 + 1; i <= LED_LENGTH / 2 + r2 / 2; i++) {
            buffer[i] = color;
        }
    }
    if (r1 % 2 == 0) {
        for(i = LED_LENGTH / 2; i >= LED_LENGTH / 2 - r1 / 2; i--) {
            buffer[i] = color;
        }
    }
}

void polar_neg(struct led *buffer, int (*f)(double), struct led color, double angle) {
    limit_angle(&angle);
    int r = f(angle);
    int i;
    
    for (i = 0; i < LED_LENGTH; i++) {
        buffer[i] = color_black;
    }
    
    /*if (r >= 0) {
        for(i = LED_LENGTH / 2; i <= LED_LENGTH / 2 + r; i++) {
            buffer[i] = color;
        }
    } else {
        for(i = LED_LENGTH / 2; i >= LED_LENGTH / 2 - r; i--) {
            buffer[i] = color;
        }
    }*/
    if (r >= 0) {
        buffer[LED_LENGTH / 2 + r] = color;
    }
    else {
        buffer[LED_LENGTH / 2 - r] = color;
    }
}

void polar_neg_d(struct led *buffer, int (*f)(double), double (*g)(double), struct led color, double angle) {
    limit_angle(&angle);
    int r = f(angle);
    double d = g(angle); 
    int i;
    for (i = 0; i < LED_LENGTH; i++) {
        buffer[i] = color_black;
    }
    
    /*if (r >= 0) {
        for(i = LED_LENGTH / 2; i <= LED_LENGTH / 2 + r; i++) {
            buffer[i] = color;
        }
    } else {
        for(i = LED_LENGTH / 2; i >= LED_LENGTH / 2 - r; i--) {
            buffer[i] = color;
        }
    }*/
    if (r >= 0) {
        buffer[LED_LENGTH / 2 + r] = color;
        if(d > 1){
            buffer[LED_LENGTH / 2 + r + 1] = color;
        }
        else if(d < -1){
            buffer[LED_LENGTH / 2 - r - 1] = color;
        }
    }
    else {
        buffer[LED_LENGTH / 2 - r] = color;
        if(d > 1){
            buffer[LED_LENGTH / 2 - r + 1] = color;
        }
        else if(d < -1){
            buffer[LED_LENGTH / 2 - r - 1] = color;
        }
    }
    
}


int spiral(double angle) {
    return angle / 5.0;
}

int cardioid(double angle) {
    double t = 1.0 - cos(angle * 3.1415 / 180.0);
    return t * 36.0;
}

int cosn(double angle) {
    double t = cos(2.0 * angle * 3.1415 / 180.0);
    return t * 36.0;
}

double d_cosn(double angle) {
    double t = -2 * sin(2.0 * angle * 3.1415 / 180.0);
    return t;
}

int line(double angle) {
    double t;
    if(angle < 1.0 || angle > 359.0) t = 1;
    else t = 1 / sin(angle * 3.1415 / 180.0);
    return t * 10.0;
}

void limit_angle(double *angle){
    while(*angle > 360.0) 
        *angle -= 360.0;
    while(*angle < 0.0) 
        *angle += 360.0;
}