#ifndef _inputdata_H_
#define _inputdata_H_

#include <stdbool.h>

extern void ShowInputData();
extern void DrawDisplayBounds();

int analog1_x, analog2_x, analog1_y, analog2_y;
int dial1 = 0, dial2 = 0;
bool switch1, switch2;

#endif