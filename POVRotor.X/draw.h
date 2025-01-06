#ifndef _draw_H_
#define _draw_H_

#include "LED.h"

#define OPP_ANGLE_CORRECTION 0

extern void limit_angle(double*);
extern void pie(led *buffer, led *pie_colors, int n, double angle);
extern void polar(led *, double (*f)(double), led, double);
extern void polar_neg(led *, double (*f)(double), led, double);
extern void polar_neg_d(led *,double (*f)(double), double (*g)(double), led, double);
extern void polar_fill(led *, double (*f)(double), led, double);
extern double spiral(double);
extern double cardioid(double);
extern double cosn(double);
extern double d_cosn(double);
extern double line(double);

#endif
