#include "LED.h"
#include "draw.h"
#include <math.h>

void pie(led *buffer, led *pie_colors, int n, double angle) {
	int i;
	limit_angle(&angle);
	int idx1 = angle / (360.0 / (double)n);
	
	for (i = 0; i < LED_LENGTH / 2; i++) {
		buffer[i] = pie_colors[idx1];
	}
	
	angle = angle + OPP_ANGLE_CORRECTION + 180.0;
	limit_angle(&angle);
	int idx2 = angle / (360.0 / (double)n);
	
	for (i = LED_LENGTH / 2 + 1; i < LED_LENGTH; i++) {
		buffer[i] = pie_colors[idx2];
	}
	buffer[LED_LENGTH / 2] = color_black;
}

void polar(led *buffer, double (*f)(double), led color, double angle) {
	limit_angle(&angle);
	double r1 = f(angle) * LED_LENGTH/2;
	
	angle = angle + OPP_ANGLE_CORRECTION + 180.0;
	limit_angle(&angle);
	double r2 = f(angle) * LED_LENGTH/2;
	
	double t;
	
	int i;
	for (i = 0; i < LED_LENGTH; i++) {
		buffer[i] = color_black;
	}
	
	t = LED_LENGTH/2 + r1;
	buffer[(int)t] = color;
	
	t = LED_LENGTH/2 - r2;
	buffer[(int)t] = color;
}

void polar_fill(led *buffer, double (*f)(double), led color, double angle) {
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

void polar_neg(led *buffer, double (*f)(double), led color, double angle) {
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

void polar_neg_d(led *buffer, double (*f)(double), double (*g)(double), led color, double angle) {
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


double spiral(double angle) {
	return angle / 5.0;
}

double cardioid(double angle) {
	double t = 1.0 - cos(angle * 3.1415 / 180.0);
	return t;
}

double cosn(double angle) {
	double t = cos(2.0 * angle * 3.1415 / 180.0);
	return t;
}

double d_cosn(double angle) {
	double t = -2 * sin(2.0 * angle * 3.1415 / 180.0);
	return t;
}

double line(double angle) {
	double t;
	if(angle < 1.0 || angle > 359.0) t = 1;
	else t = 1 / sin(angle * 3.1415 / 180.0);
	return t;
}

void limit_angle(double *angle){
	while(*angle > 360.0) *angle -= 360.0;
	while(*angle < 0.0) *angle += 360.0;
}
