#ifndef _LED_H_
#define _LED_H_

#define LED_LENGTH 96
#define BUFFER_LENGTH 140 // 96/3*4 + 12

typedef struct led {
	unsigned char red, green, blue;
} led;

extern led color_white;
extern led color_black;
extern led color_red;
extern led color_green;
extern led color_blue;
extern led color_cyan;
extern led color_magenta;
extern led color_yellow;

extern led buffer[LED_LENGTH];

extern void scaleBrightness(led*);
extern void writeLEDs(led*);
extern void writeLEDs_ISR(led*);
extern void writeLEDs_hue(led*, double);

extern void led_test_loop(unsigned char, unsigned char);

extern unsigned char LED_A_tx_buffer[BUFFER_LENGTH];
extern unsigned char LED_B_tx_buffer[BUFFER_LENGTH];
extern unsigned char LED_C_tx_buffer[BUFFER_LENGTH];

#endif
