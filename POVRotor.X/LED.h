#ifndef _LED_H_
#define _LED_H_

#define LED_LENGTH 73

struct led {
    unsigned char red, green, blue;
};

extern struct led color_white;
extern struct led color_black;
extern struct led color_red;
extern struct led color_green;
extern struct led color_blue;
extern struct led color_cyan;
extern struct led color_magenta;
extern struct led color_yellow;

extern void writeLEDs(struct led*);
extern void writeLEDs_hue(struct led*, float);

#endif