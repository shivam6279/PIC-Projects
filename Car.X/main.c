#define _XTAL_FREQ 20000000
#include <xc.h>
#include <pic16f877a.h>
#include <math.h>
#include "I2C.h"
#include "PWMDriver.h"
#include "USART.h"

#pragma config FOSC = HS  
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config BOREN = OFF
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

#define servoA_pin1 1
#define servoA_pin2 0
#define servoB_pin1 2
#define servoB_pin2 3
#define engine_pin1 4
#define engine_pin2 5

int remote_x1 = 0, remote_y1 = 0, remote_x2 = 0, remote_y2 = 0;
unsigned char dial1, dial2, receive, left_switch, right_switch;

int x1, y1, x2, y2, direction;

void pwm_init(){
    CCP1CON = 12;
    CCP2CON = 12;
    PR2 = 255;
    T2CON = 7;
    CCPR1L = 0;
    CCPR2L = 0;
}

void timer_init(){
    T0CS = 0;
    T0SE = 0;
    GIE = 1;
    PSA = 0;
    PS2 = 1;
    PS1 = 1;
    PS0 = 1;
    TMR0 = 0;
    TMR0IE = 1;
}

void interrupt ISR(){
    if(RCIF){
        receive = RCREG;
        if(receive >> 5 == 0) remote_x1 = (receive & 0x1F) - 15;
        else if(receive >> 5 == 1) remote_y1 = (receive & 0x1F) - 15;
        else if(receive >> 5 == 2) remote_x2 = (receive & 0x1F) - 15;
        else if(receive >> 5 == 3) remote_y2 = (receive & 0x1F);
        else if(receive >> 5 == 4){ 
            left_switch = (receive >> 1) & 1; 
            right_switch = receive & 1; 
        }
        else if(receive >> 5 == 5){ 
            dial1 = (receive & 0b00001100) << 2;
            dial2 = (receive & 0b00000011);
        }
        if(OERR){
            CREN = 0;
            CREN = 1;
        }
    }
    if(TMR0IF){
        TMR0IF = 0;
    }
}

void init(){
    TRISB = 0;
    TRISC = 128;
    TRISD = 0;
}

void main(){
    receive = 0;
    init();
    USART_init(111111, 1);
    i2c_init();
    pwm_driver_init(1500);
    //pwm_init();
    
    while(1){
        if(remote_y1 < 0){
            write_pwm(engine_pin1, 0);
            write_pwm(engine_pin2, (int)((float)remote_y1 / (-15.0) * 4095.0));
        }
        else{
            write_pwm(engine_pin1, (int)((float)remote_y1 / 15.0 * 4095.0));
            write_pwm(engine_pin2, 0);
        }
        
        if(remote_x1 < 0){
            write_pwm(servoA_pin1, 0);
            write_pwm(servoA_pin2, (int)((float)remote_x1 / (-15.0) * 4095.0));
            write_pwm(servoB_pin1, 0);
            write_pwm(servoB_pin2, (int)((float)remote_x1 / (-15.0) * 4095.0));
        }
        else{
            write_pwm(servoA_pin1, (int)((float)remote_x1 / 15.0 * 4095.0));
            write_pwm(servoA_pin2, 0);
            write_pwm(servoB_pin1, (int)((float)remote_x1 / 15.0 * 4095.0));
            write_pwm(servoB_pin2, 0);
        }
    }
}