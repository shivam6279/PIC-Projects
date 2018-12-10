#ifndef _LED_H_
#define _LED_H_

#define LED_LENGTH 73

struct led {
    unsigned char red, green, blue;
};

struct led color_white = {255, 255, 255};
struct led color_black = {0, 0, 0};
struct led color_red = {255, 0, 0};
struct led color_green = {0, 255, 0};
struct led color_blue = {0, 0, 255};
struct led color_cyan = {0, 255, 255};
struct led color_magenta = {255, 0, 255};
struct led color_yellow = {255, 255, 0};

extern void writeLEDs(struct led*);
extern void writeLEDs_hue(struct led*, float);

#endif