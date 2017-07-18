#define _XTAL_FREQ 20000000
#include <xc.h>
#include <pic18f4550.h>
#include <math.h>
#include "I2C.h"
#include "USART.h"
#include "MPU6050.h"

#pragma config PLLDIV = 12
#pragma config CPUDIV = OSC1_PLL2
#pragma config USBDIV = 1
#pragma config FOSC = HS
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRT = OFF
#pragma config BOR = OFF
#pragma config BORV = 2
#pragma config VREGEN = OFF
#pragma config WDT = OFF
#pragma config WDTPS = 32768
#pragma config CCP2MX = ON
#pragma config PBADEN = OFF
#pragma config LPT1OSC = OFF
#pragma config MCLRE = ON
#pragma config STVREN = OFF
#pragma config LVP = OFF
#pragma config ICPRT = OFF
#pragma config XINST = OFF
#pragma config CP0 = OFF
#pragma config CP1 = OFF
#pragma config CP2 = OFF
#pragma config CP3 = OFF
#pragma config CPB = OFF
#pragma config CPD = OFF
#pragma config WRT0 = OFF
#pragma config WRT1 = OFF
#pragma config WRT2 = OFF
#pragma config WRT3 = OFF
#pragma config WRTC = OFF
#pragma config WRTB = OFF
#pragma config WRTD = OFF
#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF
#pragma config EBTRB = OFF

#define blue PORTDbits.RD4
#define red PORTDbits.RD5
#define green PORTDbits.RD6

#define motorA_pin1 PORTCbits.RC0
#define motorA_pin2 PORTEbits.RE2
#define motorB_pin1 PORTDbits.RD0
#define motorB_pin2 PORTDbits.RD1

int x1, y1;
unsigned char receive;

void pwm_init(){
    CCP1CON = 12;
    CCP2CON = 60;
    PR2 = 74;
    T2CON = 7;
    CCPR1L = 0;
    CCPR2L = 0;
}

/*void timer_init(){
    T0CS = 0;
    T0SE = 0;
    GIE = 1;
    PSA = 0;
    PS2 = 1;
    PS1 = 1;
    PS0 = 1;
    TMR0 = 0;
    TMR0IE = 1;
}*/

void delay_ms(unsigned int x){
    unsigned int i;
    for(i = 0; i < x; i++) __delay_ms(1);
}

void init(){
    TRISB = 0;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
}

void interrupt ISR(){                 
    if(RCIF){
        receive = RCREG;
        if(receive / 32 == 0) x1 = (receive % 32) - 15;
        else if(receive / 32 == 1) y1 = (receive % 32) - 15;
        if(OERR){
            CREN = 0;
            CREN = 1;
        }
    }
    if(TMR0IF){
        TMR0IF = 0;
    }
}

void main(){
    int speed_a, speed_b;
    float x_angle, x_acc_angle;
    float sum_angle = 0, output = 0;
    x_angle = 0;
    receive = 0;
    
    init();
    i2c_init();
    USART_init(111111, 1);
    pwm_init();
    blue = 0;
    green = 0;
    red = 1;
    delay_ms(50);
    MPU6050_init();
    while(x1 < 14 || y1 > (-14));
    green = 1;
    calibrate_gyros();
    red = 0;
    //timer_init();
    x1 = 0;
    y1 = 0;
    while(1){
        get_acc();
        get_gyro();
        x_acc_angle = (atan(acc_y / sqrt(acc_z * acc_z + acc_x * acc_x)) * 57.29578) - 4;//-ve tilt forward
        x_angle = (0.9 * (x_angle + gyro_x / 100) + 0.1 * x_acc_angle);
        sum_angle += (x_angle - y1 / 3) / 30;
        output = ((x_angle - y1 / 3) * 3.5 + sum_angle * 1.0 + gyro_x * 0.05);

        speed_a = (int)output;
        speed_b = (int)output;
        
        if(x1 > 8){
            speed_a -= 50;
            speed_b += 50;
        }
        else if(x1 < (-8)){
            speed_a += 50;
            speed_b -= 50;
        }
        
        if(speed_a > 255) speed_a = 255;
        else if(speed_a < (-255)) speed_a = -255;
        
        if(speed_b > 255) speed_b = 255;
        else if(speed_b < (-255)) speed_b = -255;
        
        if(speed_a < 0){
            motorA_pin1 = 0;
            motorA_pin2 = 1;
            speed_a *= (-1);
            CCPR1L = speed_a;
        }
        else{
            motorA_pin1 = 1;
            motorA_pin2 = 0;
            CCPR1L = speed_a;
        }
        if(speed_b < 0){
            motorB_pin1 = 0;
            motorB_pin2 = 1;
            speed_b *= (-1);
            CCPR2L = speed_b;
        }
        else{
            motorB_pin1 = 1;
            motorB_pin2 = 0;
            CCPR2L = speed_b;
        }
    }
}

