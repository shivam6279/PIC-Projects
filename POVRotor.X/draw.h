#ifndef _draw_H_
#define _draw_H_

#include "LED.h"

#define OPP_ANGLE_CORRECTION 0

extern void limit_angle(double*);
extern void pie(struct led *buffer, struct led *pie_colors, int n, double angle);
extern void polar(struct led *, double (*f)(double), struct led, double);
extern void polar_neg(struct led *, double (*f)(double), struct led, double);
extern void polar_neg_d(struct led *,double (*f)(double), double (*g)(double), struct led, double);
extern void polar_fill(struct led *, double (*f)(double), struct led, double);
extern double spiral(double);
extern double cardioid(double);
extern double cosn(double);
extern double d_cosn(double);
extern double line(double);

#endif
