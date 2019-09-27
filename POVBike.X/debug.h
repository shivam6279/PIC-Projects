#ifndef _debug_H_
#define _debug_H_

#include "LED.h"

enum color {RED, GREEN, BLUE};

extern void debug_int(long int, struct led*, enum color);

#endif