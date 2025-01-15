#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include "pragma.h"
#include <string.h>
#include <inttypes.h>
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

#define RPM_TIMER_FREQ 50000
#define RPM_TIMER_DEBOUNCE RPM_TIMER_FREQ / 50
#define ANGLE_OFFSET 165

volatile unsigned long int magnet_counter = 0, speed_counter = 0, p_omega = 0, omega = 0, raw_omega = 0;
volatile unsigned char magnet_flag = 1;

volatile double time = 0.0;

led cart_image[size][size];

#define RPM_LPF 0.9

void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL7SRS) speed_timer(void) {
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

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL3SRS) LED_timer(void) {
	static double angle;
	
	IFS0bits.T4IF = 0;
	angle = 360.0 * ((double)speed_counter) / ((double)omega) - ANGLE_OFFSET;

	polar_image(buffer, cart_image, angle);
	// scaleBrightness(buffer);
	writeLEDs_ISR(buffer);
//		 writeLEDs_hue(buffer, time*50);
}

long int mag(long int a) {
	if(a < 0) {
		return -a;
	}
	return a;
}

void main() {
	int i, j;
	uint32_t k;
	float rpm;
	unsigned char gif_frame = 0, max_frames;
	
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
	
	/*while(1) {
		for(k = 0; k < 0xFFFFFF; k++) {
			i = 0;
			LED_C_tx_buffer[i++] = 0x00;
			LED_C_tx_buffer[i++] = 0x00;
			LED_C_tx_buffer[i++] = 0x00;
			LED_C_tx_buffer[i++] = 0x00;
			for(j = 0; j < LED_LENGTH/3; j++) {
				LED_C_tx_buffer[i++] = 0xE5;
				LED_C_tx_buffer[i++] = (k >> 16) & 0xFF;
				LED_C_tx_buffer[i++] = (k >> 8) & 0xFF;
				LED_C_tx_buffer[i++] = k & 0xFF;
			}
			LED_C_tx_buffer[i++] = 0xFF;
			LED_C_tx_buffer[i++] = 0xFF;
			LED_C_tx_buffer[i++] = 0xFF;
			LED_C_tx_buffer[i++] = 0xFF;
			LED_C_tx_buffer[i++] = 0xFF;
			LED_C_tx_buffer[i++] = 0xFF;
			LED_C_tx_buffer[i++] = 0xFF;
			LED_C_tx_buffer[i++] = 0xFF;
			
//			for(j = 0; j < BUFFER_LENGTH; j++) {
//				SPI2_write(LED_C_tx_buffer[j]);
//			}
//			DMACONbits.ON = 1;
//			DCH0CONbits.CHEN = 1;
//			DCH0ECONbits.CFORCE = 1;
//			SPI2_write(0);
			delay_ms(2);
		}
		
	}*/
	
	led_test_loop(1, 100);
	
	for(i = 0; i < LED_LENGTH; i++){
		buffer[i] = color_black;
	}
	for(i = 0; i < 10; i++){
		writeLEDs_ISR(buffer);
		delay_ms(1);
	}
	
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
	gif_get_frame(cart_image, 0);
	gif_frame = 1;
	
	LED_TIMER_ON = 1;
	
	StartDelayCounter();
	while(1) {
		if(ms_counter2() > 50) {
			set_ms_counter2(0);
			gif_get_frame(cart_image, gif_frame);
			gif_frame = (gif_frame + 1) % max_frames;
		}
	}
}
