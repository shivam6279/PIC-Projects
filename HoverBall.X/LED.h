#ifndef _LED_H_
#define _LED_H_

#define LED_LENGTH 138
#define BUFFER_LENGTH 564 // 138*4 + 12

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

extern struct led buffer[LED_LENGTH];;

extern void scaleBrightness(struct led*, float);
extern void writeLEDs(struct led*);
extern inline void writeLEDs_ISR(struct led*);
extern void writeLEDs_hue(struct led*, double);

extern void led_test_loop();
extern void fabulous();

#endif