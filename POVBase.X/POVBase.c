#include "pragma.h"
#include <xc.h>
#include <sys/attribs.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "pic32.h"
#include "PWM.h"
#include "BLDC.h"
#include "USART.h"
#include "bitbang_I2C.h"
#include "tones.h"
#include "EEPROM.h"
#include "SPI.h"

#define FOC_TIMER_ON T4CONbits.ON

#define SETPOINT_CENTER 40.0f

unsigned char board_id = 0;
unsigned char ramp = 0, motor_on = 0;

void __ISR_AT_VECTOR(_CHANGE_NOTICE_G_VECTOR, IPL3AUTO) button_cn(void){
    IFS1bits.CNGIF = 0;
    if(BUTTON == 0 && ms_counter3() > 250) {
        reset_ms_counter3();
        if(++mode > 2) {
            mode = 0;
        }
        if(mode == 0) {
            COIL_LED = 0;
            MOTOR_LED = 0;
            
            PDC11 = 0;
            PDC12 = PWM_MAX;
            
            motor_mode = MODE_POWER;
            ramp = 0;
            SetRPM(0);
            MotorOff();
            motor_on = 0;
            
        } else if(mode == 1) {
            COIL_LED = 1;
            MOTOR_LED = 0;
            
            PDC11 = PWM_MAX / 2;
            PDC12 = PWM_MAX / 2;    
            
            motor_mode = MODE_POWER;
            ramp = 0;
            SetRPM(0);
            MotorOff();
            motor_on = 0;
            
        } else if(mode == 2) {
            COIL_LED = 1;
            MOTOR_LED = 1;
            
            PDC11 = PWM_MAX / 2;
            PDC12 = PWM_MAX / 2;
            
            motor_mode = MODE_RPM;
            ramp = 1;
        }
    }
}

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
        return -1;
    }
    
    if(temp_buffer[0] == '-')
        flag = 1;
    
    for(i = flag, tens = 1; temp_buffer[i] != '\0'; i++, tens *= 10);
    tens /= 10;
    
    for(i = flag, ret = 0; temp_buffer[i] != '\0'; i++, tens /= 10) {
        ret += (temp_buffer[i] - '0') * tens;
    }
    
    if(flag)
        ret = -ret;
    
    return ret;
}

void main() {
    int i; 
    signed int a = 0;
    int out_int = 0;
    unsigned char mode_temp;
    float q[4];
    
    PICInit();
    
    CNENGbits.CNIEG6 = 1;
    IPC12bits.CNGIP = 3;
    IPC12bits.CNGIS = 0;
    IFS1bits.CNGIF = 0;
    IEC1bits.CNGIE = 1;
    CNCONGbits.ON = 1;
    
    QEI_init();    
    
    motor_mode = MODE_POWER;
    
    USART4_init(115200);    
    timer2_init(1000);      // ms delay
    timer3_init(4500);      // us delay
    timer4_init(10000);     // FOC
//    timer5_init(50);        // speaker  
    timer6_init(500);       // velocity
    
    EEPROM_init();            
    PwmInit();
    
    delay_ms(200);    
    MotorOff();
    
//    while(1) {
//        setPhaseVoltage(0.0125, 0);
//        delay_ms(100);
//        setPhaseVoltage(0.0125, 60);
//        delay_ms(100);
//        setPhaseVoltage(0.0125, 120);
//        delay_ms(100);
//        setPhaseVoltage(0.0125, 180);
//        delay_ms(100);
//        setPhaseVoltage(0.0125, 240);
//        delay_ms(100);
//        setPhaseVoltage(0.0125, 300);
//        delay_ms(100);
//    }
    
//    setPhaseVoltage(0.0125, 0);
//    motor_mode = MODE_OFF;
//    FOC_TIMER_ON = 1;    
//    while(1) {
//        USART4_write_float(GetPosition(), 2);
//        USART4_send('\n');
//        delay_ms(150);
//    }
     
    Write_Motor_Offset(45.5); 
    motor_zero_angle = Read_Motor_Offset();
    
    MotorOff();
    FOC_TIMER_ON = 1;    
    
    StartDelaymsCounter();
    
    float target_rpm = 1500, ramp_rpm;
    
    while(1) {
        if(ms_counter() >= 2) { 
            float deltat = (float)ms_counter() / 1000.0;
            reset_ms_counter();
            USART4_write_float(GetRPM(), 2);
            USART4_send('\n');
        }
        
        if(motor_on && (target_rpm - GetRPM())/target_rpm > 0.15) {
            COIL_LED = 0;
            MOTOR_LED = 0;
            
            PDC11 = 0;
            PDC12 = PWM_MAX;
            
            motor_mode = MODE_POWER;
            ramp = 0;
            SetRPM(0);
            MotorOff();
            motor_on = 0;
        }
        
        if(ramp) {
            for(i = 1; i <= 1000 && ramp; i++) { 
                ramp_rpm = (float)i/1000.0 * target_rpm;
                SetRPM(ramp_rpm);
                reset_ms_counter2();
                while(ms_counter2() < 2);                
                if(i > 200 && (ramp_rpm - GetRPM())/ramp_rpm > 0.5) {
                    mode = 1;
                    MOTOR_LED = 0;
                    motor_mode = MODE_POWER;
                    ramp = 0;
                    SetRPM(0);
                    MotorOff();
                    break;
                }
            }
            if(ramp) {
                motor_on = 1;
            }
            ramp = 0;
        }
    }
    
    motor_mode = MODE_RPM;
    while(1) {     
        if(rx_rdy) {       
            a = parse_rx();
            rx_rdy = 0;

            if(motor_mode == MODE_POWER) {
                SetPower((float)a / 2000.0);
            } else if(motor_mode == MODE_RPM) {
                SetRPM(a);
            } else if(motor_mode == MODE_POS) {
                SetPosition(a);
            }
        }
        if(ms_counter() >= 2) { 
            float deltat = (float)ms_counter() / 1000.0;
            reset_ms_counter();
            USART4_write_float(GetRPM(), 2);
//            USART4_send(',');
//            USART4_write_float(GetRPM_der(), 2);
            USART4_send('\n');
        }
    }
}