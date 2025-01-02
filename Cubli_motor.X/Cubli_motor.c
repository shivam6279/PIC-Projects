#include "pragma.h"
#include <xc.h>
#include <sys/kmem.h>
#include <sys/attribs.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>

#include "pic32.h"
#include "PWM.h"
#include "ADC.h"
#include "BLDC.h"
#include "diags.h"
#include "string_utils.h"
#include "USART.h"
#include "bitbang_I2C.h"
#include "tones.h"
#include "EEPROM.h"
#include "TMP1075.h"

/* 
 * UART CODES:
 * 
 * I: LED On
 * O: LED Off
 * 
 * P: Power mode
 * R: RPM Mode (PID control RPM)
 * A: Angle mode PID control angle)
 * X: Off mode
 * 
 * T: Play tone
 * D: Test Menu
 * E: Toggle auto-stop
 */

const char rttll_metroid_save[] = {"Metroid_Save:d=4,o=5,b=200:d,f,d,c,a4,p,a4"};
const char rttll_mario[] = {"Mario:d=4,o=6,b=180:8e,8e,8p,8e,8p,8c,e,g,p,g5,p"};

#define ESC 1
#define LPF_PHASE 0.8f

unsigned char board_id = 0;

void parse_rx_codes() {
	if(rx_buffer[0] == 'I') {
		LED0 = 1;
		
	} else if(rx_buffer[0] == 'O') {
		LED0 = 0;
		
	} else if(rx_buffer[0] == 'P') {                
		SetPower(0);
		mode = MODE_POWER;
		
	} else if(rx_buffer[0] == 'R') {
		ResetMotorPID() ;
		SetRPM(0);
		mode = MODE_RPM;
		
	} else if(rx_buffer[0] == 'A') {                
		ResetPosition();
		SetPosition(0);
		mode = MODE_POS;
		
	} else if(rx_buffer[0] == 'X') {
		SetPower(0);
		mode = MODE_OFF;
		MotorOff();
		
	} else if(rx_buffer[0] == 'T'){
		play_tone = 1;
		
	} else if(rx_buffer[0] == 'E'){
		auto_stop ^= 0x01;

	} else if(rx_buffer[0] == 'D'){
		diagsMenu();
		mode = MODE_POWER;
		FOC_TIMER_ON = 1;
		reset_ms_counter2();
		StartDelaymsCounter();
	}
}

int main() {
	int i, j;
	signed int a = 0;
	unsigned char mode_temp;
	unsigned char temp_buffer[RX_BUFFER_SIZE];

	PICInit();
	GPIO_init();
	ADCInit();
	init_encoder_lut();

	QEI_init();
	mode = MODE_POWER;

	USART3_init(115200);    
	timer2_init(1000);      // ms delay - P = 3
	timer3_init(100000);    // us delay - P = 7
	timer4_init(50000);     // FOC      - P = 6
	timer5_init(50);        // Speaker  - P = 2
	timer6_init(500);       // RPM      - P = 4

	EEPROM_init();
	PwmInit(96000);
	MotorOff();

//	interpolate_encoder_lut(encoder_calib_data, 32);

	delay_ms(200);
	TMP1075Init();
	
	board_id = 1;//EEPROM_read(ID_ADDR);
	LED0 = 1;
	MetroidSaveTheme(board_id);
	LED0 = 0;
	
	float pp = 0.07, phase_delay;
	while(1) {
		for(j = 0; j < 50; j++) {
			for(i = 0; i < 6; i++) {
				MotorPhase(i, 0.07);
				delay_ms(4);
			}
		}
		phase_delay = 400;
		while(1) {
			for(i = 0; i  < 6; i++) {
				MotorPhase(i, pp);
				StartDelayusCounter();
				while(us_counter() < 6);
				while(!bemf_phase(i)); // Wait for back emf to rise/fall
				phase_delay = (1.0f - LPF_PHASE) * phase_delay + LPF_PHASE * (float)us_counter();
				while(us_counter() < 2*phase_delay);
			}
			
			if(j++ > 7) {
				j = 0;
				if(pp < 0.7) {
					pp += 0.001;
				}
			}
		}
	}

	motor_zero_angle = 19.6875; //Read_Motor_Offset();

	FOC_TIMER_ON = 1;
	MotorOff();
	
	delay_ms(200);

	StartDelaymsCounter();
	while(1) {
		if(rx_rdy) {
			for(i = 0; rx_buffer[i] != '\0'; i++) {
				temp_buffer[i] = rx_buffer[i];
			}
			temp_buffer[i] = '\0';
			rx_rdy = 0;

			if(char_isAlpha(temp_buffer[0])) {
				parse_rx_codes();
			} else {
				a = str_toInt(temp_buffer);

				if(mode == MODE_POWER) {
					SetPower((float)a / 2000.0);
				} else if(mode == MODE_RPM) {
					SetRPM(a);
				} else if(mode == MODE_POS) {
					SetPosition(a);
				}
				reset_ms_counter3();
			}
		}

		if(play_tone) {
			if(mode == MODE_POWER) {
				SetPower(0);
			} else if(mode == MODE_RPM) {
				SetRPM(0);
			} else if(mode == MODE_POS) {
				INDX1CNT = 0;
				SetPosition(0);
			}
			mode_temp = mode;
			mode = MODE_OFF;
			LED0 = 1;
			MetroidSaveTheme(board_id);
			LED0 = 0;
			mode = mode_temp;
			play_tone = 0;
			StartDelaymsCounter();
		}
		
		// if(auto_stop) {
		// 	if(ms_counter3() > 1000) {
		// 		SetPower(0);
		// 	}
		// }
		
		if(ms_counter2() >= 2) {
			reset_ms_counter2();
			USART3_write_float(GetRPM(), 2);
			// USART3_send_str(", ");
			// USART3_write_float(GetRPM_der(), 2);
			USART3_send('\n');
		}
	}
}
