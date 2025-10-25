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
#include "PID.h"
#include "PWM.h"
#include "servo.h"
#include "ADC.h"
#include "BLDC.h"
#include "diags.h"
#include "string_utils.h"
#include "USART.h"
#include "SPI.h"
#include "bitbang_I2C.h"
#include "tones.h"
#include "EEPROM.h"
#include "TMP1075.h"

#define MHZ(x) x * 1000000.0f
#define KHZ(x) x * 1000.0f

//float save_data[1500][10];

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
	motor_mode mode_temp;
	unsigned char temp_buffer[RX_BUFFER_SIZE];

	PICInit();
	GPIO_init();
	ADCInit();
	init_encoder_lut();

	QEI_init();
//	SPI1_init(MHZ(10));
	mode = MODE_POWER;

	USART3_init(250000);    
	timer2_init(KHZ(1));	// ms delay		- P = 3
	timer3_init(0);			// sensorless	- P = 7
	timer4_init(KHZ(25));	// FOC			- P = 6
	timer5_init(50);		// Speaker		- P = 2
	timer6_init(500);		// RPM			- P = 4
//	timer7_init(KHZ(25));	// SPI data request
	timer8_init(0);			// Sensorless
	
	MotorPIDInit();
	
	servoInit(1500);
	
	EEPROM_init();
	PwmInit(96000);
//	MotorShort(1);
	MotorOff();
	
//	interpolate_encoder_lut(encoder_calib_data, 12);
	
	ENC_VCC = 0;
	delay_ms(200);
	TMP1075Init();
	ENC_VCC = 1;
	
//	ADCCalib();
	isns_u_offset = 1.63;
	isns_v_offset = 1.645;
	EEPROM_readAll();
	
	board_id = eeprom_board_id;
	LED0 = 1;
//	MetroidSaveTheme(board_id);
	PlayWav();
	LED0 = 0;
	
	servoOff();
	motor_zero_angle = eeprom_zero_offset;
	motor_pole_pairs = eeprom_pole_pairs;
	
	/*FOC_TIMER_ON = 0;
	float pp = 0.07;
	while(1) {
		for(j = 0; j < 10; j++) {
			for(i = 0; i < 6; i++) {
				MotorPhase(i, pp);
				delay_ms(4);
			}
		}
		
//		MotorPhase(0, pp);
//		SensorlessStart(pp);
//		while(1);
//		while(1) {
//			delay_ms(100);
//			SetPower(GetPower() + 0.001);
//			if(GetPower() > 0.5) {
//				break;
//			}
//		}
//		while(1);
		
		IEC5bits.PWM4IE = 1;
		phase_delay = 400;
		StartDelaymsCounter();
		while(1) {
			for(i = 0; i  < 6; i++) {
				MotorPhase(i, pp);
				StartDelayusCounter();
				while(TMR3 < 3600);
				while(!bemf_phase(i)); // Wait for back emf to rise/fall
				LED0 = 1;
				phase_delay = LPF_PHASE * phase_delay + (1.0f - LPF_PHASE) * (float)TMR3;
//				phase_delay = TMR3;
				while(TMR3 < 1.25*phase_delay);
				LED0 = 0;
			}
			
			if(ms_counter() > 10) {
				reset_ms_counter();
				if(pp < 1.0) {
					pp += 0.0002;
				}
			}
		}
	}*/
	
	waveform_mode = MOTOR_FOC;
	TMR7 = 50;
	FOC_TIMER_ON = 1;
//	MotorOff();
	
	/*SetPower(250 / 2000.0);
	delay_ms(1000);
	SetPower(500 / 2000.0);
	delay_ms(1000);
	SetPower(750 / 2000.0);
	delay_ms(1000);
	float vsns_u, vsns_v, vsns_w, vsns_x;
	for(i = 0; i < 1500; i++) {
//		isns_u = ((float)adc_buffer[1][0][0] * ADC_CONV_FACTOR - isns_u_offset) / 20.0f / ISNS_UVW_R;
//		isns_v = ((float)adc_buffer[4][0][0] * ADC_CONV_FACTOR - isns_v_offset) / 20.0f / ISNS_UVW_R;
//		isns_w = -(isns_u + isns_v);
		
		vsns_u = (float)adc_buffer[2][0][0] * ADC_CONV_FACTOR / MOTOR_VSNS_DIVIDER;
		vsns_v = (float)adc_buffer[3][0][0] * ADC_CONV_FACTOR / MOTOR_VSNS_DIVIDER;
		vsns_w = (float)adc_buffer[0][0][0] * ADC_CONV_FACTOR / MOTOR_VSNS_DIVIDER;
		vsns_x = (float)adc_buffer[5][0][0] * ADC_CONV_FACTOR / MOTOR_VSNS_DIVIDER;
	
		save_data[i][0] = normalizeAngle(GetPosition()*pole_pairs);
		save_data[i][1] = vsns_u;
		save_data[i][2] = vsns_v;
		save_data[i][3] = vsns_w;
		save_data[i][4] = vsns_x;
		save_data[i][5] = isns_u;
		save_data[i][6] = isns_v;
		save_data[i][7] = isns_w;
		save_data[i][8] = foc_iq;
		save_data[i][9] = foc_id;
		delay_us(40);
	}
	FOC_TIMER_ON = 0;
	MotorOff();
	for(i = 0; i < 1500; i++) {
		USART3_write_float(save_data[i][0], 2);
		USART3_send_str(", ");
		USART3_write_float(save_data[i][1], 4);
		USART3_send_str(", ");
		USART3_write_float(save_data[i][2], 4);
		USART3_send_str(", ");
		USART3_write_float(save_data[i][3], 4);
		USART3_send_str(", ");
		USART3_write_float(save_data[i][4], 4);
		USART3_send_str(", ");
		USART3_write_float(save_data[i][5], 4);
		USART3_send_str(", ");
		USART3_write_float(save_data[i][6], 4);
		USART3_send_str(", ");
		USART3_write_float(save_data[i][7], 4);
		USART3_send_str(", ");
		USART3_write_float(save_data[i][8], 4);
		USART3_send_str(", ");
		USART3_write_float(save_data[i][9], 4);
		USART3_send_str("\n");
		delay_ms(10);
	}
	while(1);*/
	
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
					pid_focIq.setpoint = (float)a / 500.0f;
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
			FOC_TIMER_ON = 1;
		}
		
		 if(auto_stop) {
		 	if(ms_counter3() > 200) {
		 		SetPower(0);
		 	}
		 }
		
		if(U3STAbits.OERR) {
			U3STAbits.OERR = 0;
		}
		
		if(ms_counter2() >= 2) {
			reset_ms_counter2();
//			USART3_write_float(GetPower(), 4);
			USART3_write_float(GetRPM(), 2);
			USART3_send_str(", ");
//			USART3_write_float(GetRPM_der(), 2);
//			USART3_send_str(", ");
			USART3_write_float(foc_iq, 5);
			USART3_send_str(", ");
			USART3_write_float(foc_id, 5);
//			USART3_send_str(", ");
//			USART3_write_float(pid_focId.output, 2);
//			USART3_send_str(", ");
//			USART3_write_float(GetPower(), 2);
//			USART3_send_str(", ");
//			USART3_write_float(pid_focId.output, 5);
//			USART3_write_float(sqrt(pid_focIq.output*pid_focIq.output + pid_focId.output*pid_focId.output), 5);
			USART3_send('\n');
		}
	}
}
