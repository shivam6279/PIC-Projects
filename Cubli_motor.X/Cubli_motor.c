#include "pragma.h"
#include <xc.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "pic32.h"
#include "PWM.h"
#include "BLDC.h"
#include "USART.h"

int parse_rx() {
    int i, ret;
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
        ret *= -1;
    
    return ret;
}

void main() {
    int i;
    unsigned char hall;
    PICInit();
    
    USART3_init(115200);    
    timer2_init(1000);       
    timer3_init(100000);    
    timer4_init(1000);
    
    PMD4bits.PWM12MD = 1;
    
    PwmInit(24000);
    
    CNCONB = 0;
    CNENBbits.CNIEB6 = 1;
    CNENBbits.CNIEB7 = 1;
    CNENBbits.CNIEB9 = 1;       
    IFS1bits.CNBIF = 0;
    IPC11bits.CNBIP = 5;
    IPC11bits.CNBIS = 0;
    IEC1bits.CNBIE = 1;    
    CNCONBbits.ON = 1;    
    motor_speed = 0;
    commutate = 1;
    current_phase = GetHallPhase();
    MotorPhase(current_phase + commutate, motor_speed);
    
    CFGCONbits.IOLOCK = 0;
    QEA4Rbits.QEA4R = 0b0100;
    QEB4Rbits.QEB4R = 0b0000;
    CFGCONbits.IOLOCK = 1;
    QEI4CON = 0;    
    QEI4IOC = 0;    
    QEI4CONbits.QEIEN = 1;
    
    //USART3_send(0x55);
    USART3_send_str("PWM resolution: ");
    USART3_write_int(PWM_MAX);
    USART3_send('\n'); 
    
    float min_speed = (float)(PWM_MAX * 500.0 / 2000.0);
    
    long int a;
    StartDelaymsCounter();
    while(1) {
        if(ms_counter() >= 50) {
            a = VEL4CNT;
            StartDelaymsCounter();
            USART3_write_int(a);
            USART3_send('\n');
//            if(a == 0 && commutate != 0) {
//                hall = GetHallPhase();
//                MotorPhase(hall + commutate, min_speed);
//            }
        }
        if(rx_rdy) {
            int a = parse_rx();
            rx_rdy = 0;
            
            if(a >= 0) 
                commutate = 1;
            else if(a < 0) 
                commutate = -1;
            else {
                commutate = 0;
                MotorPhase(0, 0);
            }
            
            a = fabs(a);
            
            motor_speed = (float)(PWM_MAX * (float)a / 2000.0);
            
            if(commutate == -1)
                USART3_send('-');
            USART3_write_int(motor_speed);
            USART3_send('\n');
        }
    }
}