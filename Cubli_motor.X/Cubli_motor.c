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
#include "USART.h"
#include "MPU6050.h"
#include "bitbang_I2C.h"
#include "tones.h"
#include "EEPROM.h"
#include "SPI.h"
#include "AHRS.h"

/* 
 * UART CODES:
 * 
 * I: LED On
 * O : LED Off
 * 
 * P: Power mode
 * R: RPM Mode (PID control RPM)
 * A: Angle mode PID control angle)
 * 
 * T: Play tone
 */

#define FOC_TIMER_ON T4CONbits.ON

#define SETPOINT_CENTER 40.0f

#define ESC 1

unsigned char board_id = 0;

void ResetQuaternion(float q[]){
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
}

signed int parse_rx() {
    int i;
    signed int ret;
    unsigned long int tens;
    unsigned char flag = 0;
    
    unsigned char temp_buffer[RX_BUFFER_SIZE];
    
    for(i = 0; rx_buffer[i] != '\0'; i++) {
        temp_buffer[i] = rx_buffer[i];
    }
    temp_buffer[i] = '\0';
    
    if((temp_buffer[0] > '9' || temp_buffer[0] < '0') && temp_buffer[0] != '-') {
        return 0.0;
    }
    
    if(temp_buffer[0] == '-') {
        flag = 1;
    }
    
    for(i = flag; rx_buffer[i] != '\0'; i++) {
        if(temp_buffer[i] > '9' || temp_buffer[i] < '0') {
            return 0.0;
        }
    }
    
    for(i = flag, tens = 1; temp_buffer[i] != '\0'; i++, tens *= 10);
    tens /= 10;
    
    for(i = flag, ret = 0; temp_buffer[i] != '\0'; i++, tens /= 10) {
        ret += (temp_buffer[i] - '0') * tens;
    }
    
    if(flag) {
        ret = -ret;
    }
    
    return ret;
}

int main() {
    int i;
    signed int a = 0;
    unsigned char mode_temp;
    
    PICInit();
    
    QEI_init();
    mode = MODE_POWER;
    
    USART3_init(115200);    
    timer2_init(1000);      // ms delay - P = 3
    timer3_init(4500);      // us delay - P = 7
    timer4_init(10000);     // FOC - P = 6
    timer5_init(50);        // speaker - P = 2
    timer6_init(500);       // velocity - P = 4
//    timer7_init(10000);     // counter
    
    EEPROM_init();
            
    PwmInit(24000);
    
    while(1) {
        LED0 = 1;
        delay_ms(200);
        LED0 = 0;
        delay_ms(200);
    }
    
    delay_ms(200);
    
    MotorOff();
    
    board_id = 1;//EEPROM_read(ID_ADDR);
    
    LED0 = 1;
    MetroidSaveTheme(board_id);
    LED0 = 0;
    
//    mode = MODE_OFF;
//    while(1) {
//        for(i = 1; i <= 6; i++) {
//            MotorPhase(i, 0.02);
//            delay_ms(100);
//        }
//    }
    
//    setPhaseVoltage(0.03, 0); 
//    FOC_TIMER_ON = 1; 
//    mode = MODE_OFF;
//    while(1) {
//        USART3_write_float(GetPosition(), 2);
//        USART3_send('\n');
//        delay_ms(150);
//    }
    
    motor_zero_angle = 0; //Read_Motor_Offset();

    FOC_TIMER_ON = 1;
    MotorOff();
     
    bool flag = false;

    StartDelaymsCounter();
    while(1) {
        if(rx_rdy) {         
            a = parse_rx();
            rx_rdy = 0;

            if(mode == MODE_POWER) {
                SetPower((float)a / 2000.0);
            } else if(mode == MODE_RPM) {
                SetRPM(a);
            } else if(mode == MODE_POS) {
                SetPosition(a);
            }
            USART3_send_str("****MESSAGE RECEIVED****\n");
            USART3_write_float(a, 2);
            USART3_send_str("\n************************\n");
        }

        if(play_tone) {
            if(mode == MODE_POWER) {
                SetPower(0);
            } else if(mode == MODE_RPM) {
                SetRPM(0);
            } else if(mode == MODE_POS) {
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
        
        if(auto_stop) {
            if(ms_counter2() > 100) {
                SetPower(0);
            }
        }
        
//        if(ms_counter2() >= 2) {
//            reset_ms_counter2();
//            USART3_send_str("test\n");
//            USART3_write_float(GetRPM(), 2);
//            USART3_send_str(", ");
//            USART3_write_float(GetRPM_der(), 2);
//            USART3_send('\n');
//        }
    }
}