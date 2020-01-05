#ifndef _draw_H_
#define _draw_H_

#include <stdbool.h>

extern void ShowInputData();
extern void DebugInfo();
extern void DrawPixel(unsigned int, unsigned int, unsigned int);
extern void WriteStr(char*, unsigned int, unsigned int, unsigned int);
extern void WriteInt(int, unsigned char, unsigned int, unsigned int, unsigned int);
extern void WriteFloat(double, unsigned char, unsigned char, unsigned int, unsigned int, unsigned int);
extern void FillRect(unsigned int, unsigned int, unsigned int, unsigned int, unsigned int);
extern void DrawLine(int, int, int, int,unsigned int);
extern void DrawCircle(int, int, int, unsigned int);
extern float mag(float);
extern float sign(float);

#endif