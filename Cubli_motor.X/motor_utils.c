#include "motor_utils.h"
#include "string_utils.h"
#include "BLDC.h"
#include "PWM.h"
#include <xc.h>
#include <stdbool.h>
#include <sys/attribs.h>
#include <math.h>
#include "USART.h"
#include "pic32.h"

#define PRINT_MENU() 	USART3_send_str("1. Spin Motor\n"); \
						USART3_send_str("2. Motor Zero\n"); \
						USART3_send_str("3. Calibrate Encoder\n"); \
						USART3_send_str("4. Display Encoder\n"); \
						USART3_send_str("x. Exit\n");

char read_rx_char() {
	unsigned char temp_buffer[RX_BUFFER_SIZE], ch = 0;
	if(rx_rdy) {
		ch = rx_buffer[0];
		rx_rdy = 0;
	}
	return ch;
}

void test_menu() {
	unsigned char temp_buffer[RX_BUFFER_SIZE];
	unsigned char ch = 0;
    unsigned int i;

	MotorOff();
	PRINT_MENU();
    
	while(ch != 'x') {
        if(rx_rdy) {
            ch = read_rx_char();

            if(ch == '1') {
            	test_spinMotor();
            	PRINT_MENU();
            } else if (ch =='2') {
            	test_setMotorZero();
            	PRINT_MENU();
            } else if (ch =='3') {
            	test_calibrateEncoder();
            	PRINT_MENU();
            } else if (ch =='4') {
            	test_printEncoder();
            	PRINT_MENU();
            } else if (char_toLower(ch) == 'h') {
            	PRINT_MENU();
            }
            
        }
    }
}

void test_spinMotor() {
    unsigned int i;
    unsigned char ch;
    
	FOC_TIMER_ON = 1; 
	mode = MODE_OFF;
	while(1) {
		for(i = 0; i < 360; i += 60) {
			setPhaseVoltage(0.03, (float)i);
			delay_ms(500);
			USART3_write_float(GetPosition(), 2);
			USART3_send('\n');
			delay_ms(250);

			if(rx_rdy) {
            	ch = read_rx_char();
            	if(ch == 'x') {
            		mode = MODE_OFF;
            		MotorOff();
            		return;
            	}
        	}
   		}
	}
}

void test_setMotorZero() {
	unsigned char ch;

	setPhaseVoltage(0.03, 0);
	motor_zero_angle = 0;
	FOC_TIMER_ON = 1; 
	mode = MODE_OFF;
	while(1) {
		USART3_write_float(GetPosition(), 2);
		USART3_send('\n');
		delay_ms(150);

		if(rx_rdy) {
        	ch = read_rx_char();
        	if(ch == 'x') {
        		mode = MODE_OFF;
        		MotorOff();
        		return;
        	}
    	}
	}	
}

void test_printEncoder() {
	unsigned char ch;

	FOC_TIMER_ON = 1; 
	mode = MODE_OFF;
	while(1) {
		USART3_write_float(GetPosition(), 2);
		USART3_send('\n');
		delay_ms(50);

		if(rx_rdy) {
        	ch = read_rx_char();
        	if(ch == 'x') {
        		mode = MODE_OFF;
        		MotorOff();
        		return;
        	}
    	}
	}	
}

void test_calibrateEncoder() {
	unsigned char ch;
	unsigned int i, j, arr_indx;
	float power = 0.03;
	float pos = GetPosition(), pre_pos;
	long int pos_cnt, ind_cnt;
	
    pos_cnt = POS1CNT;
    ind_cnt = INDX1CNT;

	FOC_TIMER_ON = 0;
	mode = MODE_OFF;
	while(1) {
		for(i = 0; i < 360; i += 60) {
			setPhaseVoltage(power, (float)i);
			delay_ms(250);
			pre_pos = pos;

			pos_cnt = POS1CNT;
    		ind_cnt = INDX1CNT;
    		pos = pos_cnt * 360.0f / ENCODER_RES;
    		while(pos < 0.0) {
		        pos += 360.0;
		    }
		    while(pos > 360.0) {
		        pos -= 360.0;
		    }

		    if(rx_rdy) {
            	ch = read_rx_char();
            	if(ch == 'x') {
            		mode = MODE_OFF;
            		MotorOff();
            		return;
            	}
        	}
		}
		pre_pos = pos;
		pos = GetPosition();
		if(pre_pos - pos > 180) {
			break;
		}
	}

	float output[(int)POLE_PAIRS*6][2];

	arr_indx = 0;
	for(j = 0; j < POLE_PAIRS; j++) {
		for(i = 0; i < 360; i += 60) {
			setPhaseVoltage(power, (float)i);
			delay_ms(250);

			pos_cnt = POS1CNT;
    		ind_cnt = INDX1CNT;
    		pos = pos_cnt * 360.0f / ENCODER_RES;
    		while(pos < 0.0) {
		        pos += 360.0;
		    }
		    while(pos > 360.0) {
		        pos -= 360.0;
		    }

			output[arr_indx][0] = 360.0/(float)POLE_PAIRS * j + i * (float)POLE_PAIRS/360.0;
			output[arr_indx][1] = pos;

			if(rx_rdy) {
            	ch = read_rx_char();
            	if(ch == 'x') {
            		mode = MODE_OFF;
            		MotorOff();
            		return;
            	}
        	}
		}
	}

	for(arr_indx = 0; arr_indx < POLE_PAIRS*6; arr_indx++) {
		USART3_write_float(output[arr_indx][0], 4);
		USART3_send(',');
		USART3_write_float(output[arr_indx][0], 4);
		USART3_send('\n');
	}
}