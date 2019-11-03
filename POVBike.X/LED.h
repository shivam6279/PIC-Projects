#ifndef _LED_H_
#define _LED_H_

#define LED_LENGTH 78
#define RADIUS_OFFSET 5

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

extern void start_frame();
extern void start_frame4();
extern void end_frame();
extern void end_frame4();

extern void writeLEDs(struct led[][LED_LENGTH + RADIUS_OFFSET]);
extern void writeLEDs_hue(struct led*, float);

#endif