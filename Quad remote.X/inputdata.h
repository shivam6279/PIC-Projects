#ifndef _inputdata_H_
#define _inputdata_H_

#include <stdbool.h>

extern void ShowInputData();
extern void DrawDisplayBounds();

extern void DecodeString(char[], float[]);
extern float StrToFloat(char[]);

extern int analog1_x, analog2_x, analog1_y, analog2_y;
extern int dial1, dial2;
extern bool switch1, switch2;

#endif