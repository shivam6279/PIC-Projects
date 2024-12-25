#include "pragma.h"
#include <xc.h>
#include <sys/kmem.h>
#include <sys/attribs.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "pic32.h"
#include "PWM.h"
#include "BLDC.h"
#include "motor_utils.h"
#include "string_utils.h"
#include "USART.h"
#include "bitbang_I2C.h"
#include "tones.h"
#include "EEPROM.h"
#include "SPI.h"

/* 
 * UART CODES:
 * 
 * I: LED On
 * O: LED Off
 * 
 * P: Power mode
 * R: RPM Mode (PID control RPM)
 * A: Angle mode PID control angle)
 * 
 * T: Play tone
 * D: Test Menu
 * E: Toggle auto-stop
 */

#define ESC 1

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
        
    } else if(rx_buffer[0] == 'T'){
        play_tone = 1;
        
    } else if(rx_buffer[0] == 'E'){
        auto_stop ^= 0x01;

    } else if(rx_buffer[0] == 'D'){
        test_menu();
        FOC_TIMER_ON = 1;
        reset_ms_counter2();
        StartDelaymsCounter();
    }
}

int main() {
    int i;
    signed int a = 0;
    unsigned char mode_temp;
    unsigned char temp_buffer[RX_BUFFER_SIZE];
    
    PICInit();
    GPIO_init();
    init_encoder_lut();
    
    QEI_init();
    mode = MODE_POWER;
    
    USART3_init(115200);    
    timer2_init(1000);      // ms delay - P = 3
    timer3_init(4500);      // us delay - P = 7
    timer4_init(50000);     // FOC - P = 6
    timer5_init(50);        // speaker - P = 2
    timer6_init(500);       // velocity - P = 4
//    timer7_init(10000);     // counter
    
    EEPROM_init();
    PwmInit(96000);
    MotorOff();
    
//    interpolate_encoder_lut(POLE_PAIRS*6, encoder_calib_data);

    delay_ms(200);

    board_id = 1;//EEPROM_read(ID_ADDR);
    LED0 = 1;
    MetroidSaveTheme(board_id);
    LED0 = 0;
    
    motor_zero_angle = 41.8359; //Read_Motor_Offset();

    FOC_TIMER_ON = 1;
    MotorOff();

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
                    INDX1CNT = 0;
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
        
//        if(auto_stop) {
//            if(ms_counter3() > 1000) {
//                SetPower(0);
//            }
//        }
        
        if(ms_counter2() >= 2) {
            reset_ms_counter2();
            USART3_write_float(GetRPM(), 2);
//            USART3_send_str(", ");
//            USART3_write_float(GetRPM_der(), 2);
            USART3_send('\n');
        }
    }
}