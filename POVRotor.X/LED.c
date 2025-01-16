#include "SPI.h"
#include "LED.h"
#include "draw.h"
#include <math.h>
#include <xc.h>
#include "pic32.h"
#include <sys/attribs.h>  

#define LED_BRIGTHNESS 0.5
static unsigned char br_byte = 0b11100000 | (unsigned char)((float)LED_BRIGTHNESS*31);

static const int led_length_2 = LED_LENGTH / 2;

led color_white = {255, 255, 255};
led color_black = {0, 0, 0};
led color_red = {255, 0, 0};
led color_green = {0, 255, 0};
led color_blue = {0, 0, 255};
led color_cyan = {0, 255, 255};
led color_magenta = {255, 0, 255};
led color_yellow = {255, 255, 0};

led buffer[LED_LENGTH];

unsigned char __attribute__ ((coherent, aligned(8))) LED_A_tx_buffer[BUFFER_LENGTH];
unsigned char __attribute__ ((coherent, aligned(8))) LED_B_tx_buffer[BUFFER_LENGTH];
unsigned char __attribute__ ((coherent, aligned(8))) LED_C_tx_buffer[BUFFER_LENGTH];
static float led_buffer_scaler[LED_LENGTH] = {1, 0.985263158, 0.970526316, 0.955789474, 0.941052632, 0.926315789, 0.911578947, 0.896842105, 0.882105263, 0.867368421, 0.852631579, 0.837894737, 0.823157895, 0.808421053, 0.793684211, 0.778947368, 0.764210526, 0.749473684, 0.734736842, 0.72, 0.705263158, 0.690526316, 0.675789474, 0.661052632, 0.646315789, 0.631578947, 0.616842105, 0.602105263, 0.587368421, 0.572631579, 0.557894737, 0.543157895, 0.528421053, 0.513684211, 0.498947368, 0.484210526, 0.469473684, 0.454736842, 0.44, 0.425263158, 0.410526316, 0.395789474, 0.381052632, 0.366315789, 0.351578947, 0.336842105, 0.322105263, 0.307368421, 0.3, 0.314736842, 0.329473684, 0.344210526, 0.358947368, 0.373684211, 0.388421053, 0.403157895, 0.417894737, 0.432631579, 0.447368421, 0.462105263, 0.476842105, 0.491578947, 0.506315789, 0.521052632, 0.535789474, 0.550526316, 0.565263158, 0.58, 0.594736842, 0.609473684, 0.624210526, 0.638947368, 0.653684211, 0.668421053, 0.683157895, 0.697894737, 0.712631579, 0.727368421, 0.742105263, 0.756842105, 0.771578947, 0.786315789, 0.801052632, 0.815789474, 0.830526316, 0.845263158, 0.86, 0.874736842, 0.889473684, 0.904210526, 0.918947368, 0.933684211, 0.948421053, 0.963157895, 0.977894737, 0.992631579};

void start_frame2() {
	SPI2_write(0);
	SPI2_write(0);
	SPI2_write(0);
	SPI2_write(0);
}

void start_frame3() {
	SPI3_write(0);
	SPI3_write(0);
	SPI3_write(0);
	SPI3_write(0);
}

void start_frame4() {
	SPI4_write(0);
	SPI4_write(0);
	SPI4_write(0);
	SPI4_write(0);
}

void start_frame_all() {
	SPI_all_write(0, 0, 0);
	SPI_all_write(0, 0, 0);
	SPI_all_write(0, 0, 0);
	SPI_all_write(0, 0, 0);
}

void end_frame2() {
	SPI2_write(255);
	SPI2_write(255);
	SPI2_write(255);
	SPI2_write(255);
	
	SPI2_write(255);
	SPI2_write(255);
	SPI2_write(255);
	SPI2_write(255);
}

void end_frame3() {
	SPI3_write(255);
	SPI3_write(255);
	SPI3_write(255);
	SPI3_write(255);
	
	SPI3_write(255);
	SPI3_write(255);
	SPI3_write(255);
	SPI3_write(255);
}

void end_frame4() {
	SPI4_write(255);
	SPI4_write(255);
	SPI4_write(255);
	SPI4_write(255);
	
	SPI4_write(255);
	SPI4_write(255);
	SPI4_write(255);
	SPI4_write(255);
}

void end_frame_all() {
	SPI_all_write(255, 255, 255);
	SPI_all_write(255, 255, 255);
	SPI_all_write(255, 255, 255);
	SPI_all_write(255, 255, 255);
	
	SPI_all_write(255, 255, 255);
	SPI_all_write(255, 255, 255);
	SPI_all_write(255, 255, 255);
	SPI_all_write(255, 255, 255);
}

void LED_frame2(unsigned char red, unsigned char green, unsigned char blue) {
	SPI2_write(br_byte);
	SPI2_write(blue);
	SPI2_write(green);
	SPI2_write(red);
}

void LED_frame3(unsigned char red, unsigned char green, unsigned char blue) {
	SPI3_write(br_byte);
	SPI3_write(blue);
	SPI3_write(green);
	SPI3_write(red);
}

void LED_frame4(unsigned char red, unsigned char green, unsigned char blue) {
	SPI4_write(br_byte);
	SPI4_write(blue);
	SPI4_write(green);
	SPI4_write(red);
}

void LED_frame_all(unsigned char red2, unsigned char green2, unsigned char blue2, unsigned char red3, unsigned char green3, unsigned char blue3, unsigned char red4, unsigned char green4, unsigned char blue4) {
	SPI_all_write(br_byte, br_byte, br_byte);
	SPI_all_write(blue2, blue3, blue4);
	SPI_all_write(green2, green3, green4);
	SPI_all_write(red2, red3, red4);
}

void scaleBrightness(led *buffer) {
	int i;
//    float r, scale;
//    float min_scale_1 = 1.0 - min_scale;
	
	for(i = 0; i < LED_LENGTH; i++) {
		buffer[i].red = (float)((float)buffer[i].red * led_buffer_scaler[i]);
		buffer[i].blue = (float)((float)buffer[i].blue * led_buffer_scaler[i]);
		buffer[i].green = (float)((float)buffer[i].green * led_buffer_scaler[i]);
	}
}

void writeLEDs(led *buffer) {
	unsigned int i;
	
	start_frame_all();
	for(i = 0; i < 33; i++) {
		LED_frame_all(buffer[i].red, buffer[i].green, buffer[i].blue, buffer[i+32].red, buffer[i+32].green, buffer[i+32].blue, buffer[i+64].red, buffer[i+64].green, buffer[i+64].blue);
	}
	end_frame_all();
}

inline void writeLEDs_ISR(led *buffer) {
	unsigned int i, j;
	
	for(i = 0; i < 4; i++) {
		LED_A_tx_buffer[i] = 0;
		LED_B_tx_buffer[i] = 0;
		LED_C_tx_buffer[i] = 0;
	}
	
	for(j = 0, i = 4; j < LED_LENGTH/3; j++) {
		LED_A_tx_buffer[i++] = br_byte;
		LED_A_tx_buffer[i++] = buffer[j].blue;
		LED_A_tx_buffer[i++] = buffer[j].green;
		LED_A_tx_buffer[i++] = buffer[j].red;
	}
	for(i = 4; j < (LED_LENGTH/3)*2; j++) {
		LED_B_tx_buffer[i++] = br_byte;
		LED_B_tx_buffer[i++] = buffer[j].blue;
		LED_B_tx_buffer[i++] = buffer[j].green;
		LED_B_tx_buffer[i++] = buffer[j].red;
	}
	for(i = 4; j < LED_LENGTH; j++) {
		LED_C_tx_buffer[i++] = br_byte;
		LED_C_tx_buffer[i++] = buffer[j].blue;
		LED_C_tx_buffer[i++] = buffer[j].green;
		LED_C_tx_buffer[i++] = buffer[j].red;
	}
	
	for(j = 0; j < 8; j++, i++) {
		LED_A_tx_buffer[i] = 255;
		LED_B_tx_buffer[i] = 255;
		LED_C_tx_buffer[i] = 255;
	}
	
	DCH0CONbits.CHEN = 1;
	DCH0ECONbits.CFORCE = 1;
	
	DCH1CONbits.CHEN = 1;
	DCH1ECONbits.CFORCE = 1;
	
	DCH2CONbits.CHEN = 1;
	DCH2ECONbits.CFORCE = 1;
}

void writeLEDs_hue(led *buffer, double hue) {
	int i;
	
	limit_angle(&hue);
	float r, g, b;
	float cosA = cos(hue*3.14159265f / 180.0); 
	float sinA = sin(hue*3.14159265f / 180.0);
	float matrix[3][3] = {
		{ cosA + (1.0f - cosA) / 3.0f,							1.0f/3.0f * (1.0f - cosA) - sqrtf(1.0f/3.0f) * sinA,	1.0f/3.0f * (1.0f - cosA) + sqrtf(1.0f/3.0f) * sinA	},
		{ 1.0f/3.0f * (1.0f - cosA) + sqrtf(1.0f/3.0f) * sinA,	cosA + 1.0f/3.0f*(1.0f - cosA),							1.0f/3.0f * (1.0f - cosA) - sqrtf(1.0f/3.0f) * sinA	},
		{ 1.0f/3.0f * (1.0f - cosA) - sqrtf(1.0f/3.0f) * sinA,	1.0f/3.0f * (1.0f - cosA) + sqrtf(1.0f/3.0f) * sinA,	cosA + 1.0f/3.0f * (1.0f - cosA)					}
	};
	
	start_frame2();
	for(i = 0; i < LED_LENGTH; i++){
		r = buffer[i].red*matrix[0][0] + buffer[i].green*matrix[0][1] + buffer[i].blue*matrix[0][2];
		g = buffer[i].red*matrix[1][0] + buffer[i].green*matrix[1][1] + buffer[i].blue*matrix[1][2];
		b = buffer[i].red*matrix[2][0] + buffer[i].green*matrix[2][1] + buffer[i].blue*matrix[2][2];
		
		if(r < 0) {
			r = 0;
		} else if (r > 255) {
			r = 255;
		}
		if(g < 0) {
			g = 0;
		} else if (g > 255) {
			g = 255;
		}
		if(b < 0) {
			b = 0;
		} else if (b > 255) {
			b = 255;
		}
		
		LED_frame2((unsigned char)r, (unsigned char)g, (unsigned char)b);
	}
	end_frame2();
}

void led_test_loop(unsigned char cycles, unsigned char duration_ms) {
	int i, j;
	
	int d = (float)duration_ms/4.0;
	
	if(cycles == 0) {
		while(1) {
			for(j = 0; j < LED_LENGTH; j++) {
				buffer[j] = color_red;
			}
			for(j = 0; j < d; j++) {
				writeLEDs(buffer);
				delay_ms(4);
			}

			for(j = 0; j < LED_LENGTH; j++) {
				buffer[j] = color_blue;
			}
			for(j = 0; j < d; j++) {
				writeLEDs(buffer);
				delay_ms(4);
			}

			for(j = 0; j < LED_LENGTH; j++) {
				buffer[j] = color_green;
			}
			for(j = 0; j < d; j++) {
				writeLEDs(buffer);
				delay_ms(4);
			}
		}
	}
	
	for(i = 0; i < cycles; i++) {
		for(j = 0; j < LED_LENGTH; j++) {
			buffer[j] = color_red;
		}
		for(j = 0; j < d; j++) {
			writeLEDs(buffer);
			delay_ms(4);
		}

		for(j = 0; j < LED_LENGTH; j++) {
			buffer[j] = color_blue;
		}
		for(j = 0; j < d; j++) {
			writeLEDs(buffer);
			delay_ms(4);
		}

		for(j = 0; j < LED_LENGTH; j++) {
			buffer[j] = color_green;
		}
		for(j = 0; j < d; j++) {
			writeLEDs(buffer);
			delay_ms(4);
		}
	}
}
