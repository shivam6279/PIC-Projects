#include "diags.h"

#include <xc.h>
#include <stdbool.h>
#include <sys/attribs.h>
#include <math.h>

#include "pic32.h"
#include "ADC.h"
#include "USART.h"
#include "string_utils.h"
#include "BLDC.h"
#include "PWM.h"
#include "TMP1075.h"
#include "bitbang_I2C.h"

void printDiagsMenu();
char read_rx_char();

void diags_spinMotor(char*);
void diags_setMotorZero(char*);
void diags_calibrateEncoder(char*);
void diags_motor(char*);
void diags_encoder(char*);
void diags_readTemperature(char*);
void diags_readADC(char*);
void diags_i2c(char*);

typedef void (*diags_function)(char*);

typedef struct diags_menu_item {
	char name[100];
	char cmd[10];
	diags_function func;
} diags_menu_item;

const diags_menu_item diags_list[] = {
	[0] = { .name = "Spin Motor",			.cmd = "1",		.func =  diags_spinMotor},
	[1] = { .name = "Motor Zero",			.cmd = "2",		.func =  diags_setMotorZero },
	[2] = { .name = "Calibrate Encoder",	.cmd = "3",		.func =  diags_calibrateEncoder },
	[3] = { .name = "Motor Control",		.cmd = "bldc",	.func =  diags_motor },
	[4] = { .name = "Rotary Encoder",		.cmd = "enc",	.func =  diags_encoder },
	[5] = { .name = "Read Temperature",		.cmd = "temp",	.func =  diags_readTemperature },
	[6] = { .name = "Read ADCs",            .cmd = "adc",	.func =  diags_readADC },
	[7] = { .name = "I2C",            		.cmd = "i2c",	.func =  diags_i2c },
};

unsigned char diags_list_len = sizeof(diags_list) / sizeof(diags_list[0]);

void diagsMenu() {
	unsigned char input[RX_BUFFER_SIZE];
	unsigned char ch = 0;
    unsigned char i;
    unsigned int input_len;
    bool flag;

	MotorOff();
	printDiagsMenu();
    
	while(ch != 'x') {
        if(rx_rdy) {
        	for(i = 0; rx_buffer[i] != '\0'; i++) {
                input[i] = rx_buffer[i];
            }
            input[i] = '\0';
            rx_rdy = 0;
            
            ch = input[0];
            
            if(!flag) {
                USART3_send_str("[diags]: ");
                flag = false;
            }
            USART3_send_str(input);
            USART3_send('\n');

            str_removeChar(input, '\r');
            str_removeChar(input, '\n');

            if(str_isEqual(input, "help")) {
            	printDiagsMenu();

            } else if(ch == 'x') {
                mode = MODE_OFF;
                MotorOff();
                return;

            } else {
                flag = false;
                for(i = 0; i < diags_list_len; i++) {
                    if(str_beginsWith(input, diags_list[i].cmd)) {
                    	input_len = str_len(input);
                        if(input[input_len] == ' ' || input[input_len] == '\0') {
                            diags_list[i].func(input);
                            USART3_send_str("Done\n");
                            USART3_send_str("[diags]: ");
                            flag = true;
                        }
                    }
                }
               // if(!flag) {
               //     USART3_send_str("Incorrect diags command");
               // }
            }
        }
    }
}

char read_rx_char() {
	unsigned char ch = 0;
	if(rx_rdy) {
		ch = rx_buffer[0];
		rx_rdy = 0;
	}
	return ch;
}

void printDiagsMenu() {
	unsigned char i;
	for(i = 0; i < diags_list_len; i++) {
		USART3_send_str(diags_list[i].cmd);
		USART3_send_str(" - ");
		USART3_send_str(diags_list[i].name);
		USART3_send('\n');
	}
	USART3_send_str("x. Exit\n");
}

void diags_spinMotor(char *cmd) {
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

void diags_setMotorZero(char *cmd) {
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

void diags_motor(char *cmd) {
	unsigned char ch;
	static float power = 0.03;
	char arg_val[20];
    
    const char help_str[] = "\
Commands to control the BLDC motor with the parameters:\n\
-e [x]: Move motor to electrical degree x\n\
--off : Turn motor off (coast)\n\
--setpower [x] : Set power level for other commands (-1.0 to 1.0)\n";
    
    if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
        USART3_send_str(help_str);
        return;
    }

    // Set electrical angle
    if(str_getArgValue(cmd, "-e", arg_val)) {
        USART3_send_str("Motor set to electrical angle: ");
        USART3_write_float(str_toFloat(arg_val), 4);
        USART3_send_str("\nAt power: ");
        USART3_write_float(power, 2);
        USART3_send('\n');
    	mode = MODE_OFF;
    	setPhaseVoltage(power, str_toFloat(arg_val));

    } else if(str_getArgValue(cmd, "--off", arg_val)) {
        USART3_send_str("Motor off\n");
    	mode = MODE_OFF;
    	MotorOff();

    } else if(str_getArgValue(cmd, "--setpower", arg_val)) {
        power = str_toFloat(arg_val);
        USART3_send_str("Power set to: ");
        USART3_write_float(power, 4);
        USART3_send('\n');
    } else {
        USART3_send_str(help_str);
        return;
    }
}

void diags_encoder(char *cmd) {
	unsigned char ch;
	long int pos_cnt, ind_cnt;
	float pos;
    char arg_val[5];
    
    const char help_str[] = "\
Read rotary encoder data:\n\
-r : Output raw data scaled to degrees. No calibration or offset applied.\n";
    
    if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
        USART3_send_str(help_str);
        return;
    }
	
	// Display raw
	if(str_getArgValue(cmd, "-r", arg_val)) {
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

	} else {
		mode = MODE_OFF;
	    MotorOff();
	    FOC_TIMER_ON = 1; 
		while(1) {
			USART3_write_float(GetPosition(), 2);
			USART3_send_str(", ");
			USART3_write_float(GetRPM(), 2);
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
}

void diags_calibrateEncoder(char *cmd) {
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

void diags_readTemperature(char *cmd) {
	float t1 = 0, t2 = 0;
	int16_t t1_raw = 0, t2_raw = 0;
	char arg_val[5];
    
    const char help_str[] = "\
Read temp sensor TMP1075:\n\
2 sensors on device near: MCU and FETs\
-r : Output raw data\n";
    
    if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
        USART3_send_str(help_str);
        return;
    }

    if(str_getArgValue(cmd, "-r", arg_val)) {
    	TMP1075_getRawTemp(0, &t1_raw);
    	TMP1075_getRawTemp(1, &t2_raw);

    	USART3_send_str("0: "); 
		USART3_write_float(t1_raw, 2);
		USART3_send('\n'); 

		USART3_send_str("1: "); 
		USART3_write_float(t2_raw, 2);
		USART3_send('\n'); 

    } else {
		TMP1075_getTemp(0, &t1);
		TMP1075_getTemp(1, &t2);

		USART3_send_str("MCU: "); 
		USART3_write_float(t2, 2);
		USART3_send_str(" C\n"); 

		USART3_send_str("FETs: "); 
		USART3_write_float(t1, 2);
		USART3_send_str(" C\n"); 
	}
}

void diags_readADC(char *cmd) {
    float isns_u, isns_v, isns_w, isns_vbat, vsns_vbat, vsns_12v;
    char arg_val[5];
    
    const char help_str[] = "\
Read ADC data\n\
-r : Output raw data. No scaling applied.\n";
    
    if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
        USART3_send_str(help_str);
        return;
    }

	adc_readAll();

	if(str_getArgValue(cmd, "-r", arg_val)) {
		USART3_send_str("ADC1: "); 
		USART3_write_int(adc_data[0]);
		USART3_send('\n');

		USART3_send_str("ADC3: "); 
		USART3_write_int(adc_data[1]);
		USART3_send('\n');

		USART3_send_str("ADC4: "); 
		USART3_write_int(adc_data[2]);
		USART3_send('\n');

		USART3_send_str("AN7: "); 
		USART3_write_int(adc_data[3]);
		USART3_send('\n');
	    
	    USART3_send_str("AN27: "); 
		USART3_write_int(adc_data[4]);
		USART3_send('\n');
	} else {
	    isns_u = ((float)adc_data[0] * ADC_CONV_FACTOR - 1.65) / 50 / ISNS_UVW_R;
	    isns_v = ((float)adc_data[2] * ADC_CONV_FACTOR - 1.65) / 50 / ISNS_UVW_R;
	    isns_w = -(isns_u + isns_v);
	    
	    isns_vbat = (float)adc_data[4] * ADC_CONV_FACTOR / 50 / ISNS_VBAT_R;
	    vsns_vbat = (float)adc_data[3] * ADC_CONV_FACTOR / VSNS_VBAT_DIVIDER;
	    vsns_12v = (float)adc_data[1] * ADC_CONV_FACTOR / VSNS_12V_DIVIDER;
	    
		USART3_send_str("ISNS U: "); 
		USART3_write_float(isns_u, 2);
		USART3_send_str(" mA\n");

		USART3_send_str("ISNS V: "); 
		USART3_write_float(isns_v, 2);
		USART3_send_str(" mA\n");

		USART3_send_str("ISNS W: "); 
		USART3_write_float(isns_w, 2);
		USART3_send_str(" mA\n");
	    
	    USART3_send_str("VSNS VBAT: "); 
		USART3_write_float(vsns_vbat, 2);
		USART3_send_str(" V\n");
	    
	    USART3_send_str("ISNS VBAT: "); 
		USART3_write_float(isns_vbat, 2);
		USART3_send_str(" mA\n");
	    
	    USART3_send_str("VSNS 12V: "); 
		USART3_write_float(vsns_12v, 2);
		USART3_send_str(" V\n");
	}
}

void diags_i2c(char *cmd) {
    unsigned int i;
    char arg_val[25];
    char out_str[10];
    
    const char help_str[] = "\
I2C commands\n\
-s : Scan bus and report addresses found\n";
    
    if(str_getArgValue(cmd, "-h", arg_val) || str_getArgValue(cmd, "--help", arg_val)) {
        USART3_send_str(help_str);
        return;
    }
    
    // I2C scan
    if(str_getArgValue(cmd, "-s", arg_val)) {
        for(i = 1; i < 0x7F; i++) {
            if(I2C_CheckAddress(i)) {
                hexToStr(out_str, i);
                USART3_send_str(out_str);
                USART3_send('\n');
            }
        }
    }
}
