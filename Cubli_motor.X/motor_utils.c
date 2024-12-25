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

#define PRINT_MENU() 	USART3_send_str("Diags Menu\n"); \
                        USART3_send_str("1. Spin Motor\n"); \
						USART3_send_str("2. Motor Zero\n"); \
						USART3_send_str("3. Calibrate Encoder\n"); \
						USART3_send_str("4. Display Encoder\n"); \
                        USART3_send_str("5. Display Raw Encoder\n"); \
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
            
            USART3_send(ch);
            USART3_send('\n');

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
            } else if (ch =='5') {
            	test_printRawEncoder();
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
    float pos;
	long int pos_cnt, ind_cnt;
	
	motor_zero_angle = 0;
	FOC_TIMER_ON = 0;
    
    setPhaseVoltage(0.03, 0);
	while(1) {
        pos_cnt = POS1CNT;
        ind_cnt = INDX1CNT;
        pos = pos_cnt * 360.0f / ENCODER_RES;
        
		USART3_write_float(pos, 2);
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
	
	mode = MODE_OFF;
    MotorOff();
    FOC_TIMER_ON = 1; 
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

void test_printRawEncoder() {
	unsigned char ch;
    float pos;
	long int pos_cnt, ind_cnt;

	FOC_TIMER_ON = 0; 
	mode = MODE_OFF;
    MotorOff();
	while(1) {
        pos_cnt = POS1CNT;
        ind_cnt = INDX1CNT;
        pos = pos_cnt * 360.0f / ENCODER_RES;
        
		USART3_write_float(pos, 2);
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
	float power = 0.05;
	float pos = GetPosition(), pre_pos;
	long int pos_cnt, ind_cnt;
	
    pos_cnt = POS1CNT;
    ind_cnt = INDX1CNT;

	FOC_TIMER_ON = 0;
	mode = MODE_OFF;
	while(1) {
		for(i = 0; i <= 360; i += 60) {
			setPhaseVoltage(power, (float)i);
			delay_ms(100);
            
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
		
        pos_cnt = POS1CNT;
        ind_cnt = INDX1CNT;
        pos = pos_cnt * 360.0f / ENCODER_RES;
        while(pos < 0.0) {
            pos += 360.0;
        }
        while(pos > 360.0) {
            pos -= 360.0;
        }
        
		if(pre_pos - pos > 180) {
			break;
		}
	}

	float output[(int)POLE_PAIRS*6][2];

	arr_indx = 0;
	for(j = 0; j < POLE_PAIRS; j++) {
		for(i = 0; i < 360; i += 60) {
			setPhaseVoltage(power, (float)i);
			delay_ms(500);

			pos_cnt = POS1CNT;
    		ind_cnt = INDX1CNT;
    		pos = pos_cnt * 360.0f / ENCODER_RES;
    		while(pos < 0.0) {
		        pos += 360.0;
		    }
		    while(pos > 360.0) {
		        pos -= 360.0;
		    }

			output[arr_indx][0] = (360.0 * j + i) / (float)POLE_PAIRS;
			output[arr_indx][1] = pos;
            arr_indx++;

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
    
    float zero_offset = output[0][1];
    
    for(arr_indx = 0; arr_indx < (POLE_PAIRS*6); arr_indx++) {
        output[arr_indx][1] -= zero_offset;
        while(output[arr_indx][1] < 0.0) {
            output[arr_indx][1] += 360.0;
        }
        while(output[arr_indx][1] > 360.0) {
            output[arr_indx][1] -= 360.0;
        }
    }
    
    USART3_write_float(zero_offset, 4);
    USART3_send('\n');

	for(arr_indx = 0; arr_indx < (POLE_PAIRS*6); arr_indx++) {
		USART3_write_float(output[arr_indx][0], 4);
		USART3_send(',');
		USART3_write_float(output[arr_indx][1], 4);
		USART3_send('\n');
	}
    
    mode = MODE_OFF;
    MotorOff();
}