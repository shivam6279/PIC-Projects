#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include "pragma.h"
#include <string.h>
#include <sys/attribs.h>
#include "pic32.h"
#include "USART.h"
#include "SPI.h"
#include "LED.h"
#include "debug.h"
#include "draw.h"
#include "animation.h"

void delay_ms(unsigned int x);
void fabulous();
void morph();

#define RPM_TIMER_FREQ 25000 //25000
#define RPM_TIMER_DEBOUNCE 500
#define ANGLE_OFFSET 165

volatile unsigned long int magnet_counter = 0, speed_counter = 0, p_omega = 0, omega = 0, raw_omega = 0;
volatile unsigned char magnet_flag = 1;

volatile double time = 0.0;

struct led **cart_image;
struct led cart_image1[size][size], cart_image2[size][size];
struct led *temp_pointer;

#define RPM_LPF 0.9

void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL7SOFT) speed_timer(void) {
	IFS0bits.T3IF = 0;
	
	speed_counter++;
	magnet_counter++;
	
	if(PORTCbits.RC13 == 0 && magnet_flag == 1) {        
//        omega = (float)((float)speed_counter * (1.0-RPM_LPF) + (float)p_omega * RPM_LPF);
		omega = speed_counter;
		p_omega = omega;
		
		speed_counter = 0;
		magnet_counter = 0;
		magnet_flag = 0;
	}
	else if(PORTCbits.RC13 == 1 && magnet_counter > RPM_TIMER_DEBOUNCE) {        
		magnet_flag = 1;
	}
	
	time += 0.00002;
}

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL3SOFT) LED_timer(void) {
	static double angle;
	
	IFS0bits.T4IF = 0;
	if(!(LED_A_TX_INTERRUPT || LED_B_TX_INTERRUPT || LED_C_TX_INTERRUPT)) {
		angle = 360.0 * ((double)speed_counter) / ((double)omega) - ANGLE_OFFSET;
		
		polar_image(buffer, cart_image, angle);
		// scaleBrightness(buffer);
		writeLEDs_ISR(buffer);
		// writeLEDs_hue(buffer, time*50); 
	}
}

long int mag(long int a) {
	if(a < 0) {
		return -a;
	}
	return a;
}

void main() {
	int i, j, k;
	float rpm;
	unsigned char gif_frame = 0, max_frames;
	struct led temp_color;
	
	PICInit();
	TRISCbits.TRISC13 = 1;
	
	timer2_init(1000); 
	timer3_init(RPM_TIMER_FREQ);
	timer4_init(5000);
	
	timer5_init(1000000);
	
	RPM_TIMER_ON = 1;
	
	delay_ms(200);
	SPI2_init();
	SPI3_init();
	SPI4_init();
	delay_ms(200);
	
	led_test_loop(1, 100);
	
	for(j = 0; j < LED_LENGTH; j++){
		buffer[j] = color_black;
	}
	writeLEDs_ISR(buffer);
	
	rpm = 0.0;
	do{
		if(omega != 0.0) {
			rpm = (double)RPM_TIMER_FREQ / (double)omega * 60.0;
		}
		delay_ms(50);
	}while(rpm < 100.0);
	
//    for(i = 0; i < size; i++){
//        for(j = 0; j < size; j++){
//            cart_image[i][j].red =   ppm[i*size*3 + j*3];
//            cart_image[i][j].green = ppm[i*size*3 + j*3 + 1];
//            cart_image[i][j].blue =  ppm[i*size*3 + j*3 + 2];
//        }
//    }
	
	max_frames = gif_init();    
	gif_get_frame(cart_image2, 0);
	memcpy(&cart_image, &cart_image2, sizeof(cart_image2));
	gif_frame = 1;
	
	LED_TIMER_ON = 1;
//    uS_TIMER_ON = 1;
	
	unsigned char pointer_cnt = 0;
	StartDelayCounter();
	while(1) {
		if(ms_counter2() > 0) {
			set_ms_counter2(0);
			if(pointer_cnt) {
				gif_get_frame(cart_image1, gif_frame);
				cart_image = cart_image1;
			} else {
				gif_get_frame(cart_image2, gif_frame);
				cart_image = cart_image2;
			}
			pointer_cnt = pointer_cnt ^ 0x01;
			// memcpy(&cart_image, &cart_image2, sizeof(cart_image2));
			gif_frame = (gif_frame + 1) % max_frames;
		}
	}
}
