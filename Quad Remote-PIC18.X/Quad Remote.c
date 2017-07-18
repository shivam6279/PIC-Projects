#define _XTAL_FREQ 48000000
#include <xc.h>
#include <pic18f4550.h>
#include <math.h>
#include "USART.h"
#include "SPI.h"
#include "ColorLCD.h"
//#include "I2C.h"
//#include "OLED.h"
//#include "MPU6050.h"

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

#define x1_offset 473
#define y1_offset 505 
#define x2_offset 495
#define y2_offset 180

#define x1_pin 3 
#define y1_pin 2
#define x2_pin 1
#define y2_pin 0
#define dial1_pin 4
#define dial2_pin 5
#define switch1 PORTDbits.RD3
#define switch2 PORTDbits.RD2 
#define led_red PORTBbits.RB2
#define led_blue PORTBbits.RB3 
#define led_green PORTBbits.RB4

unsigned char counter =  0, x1_send, y1_send, x2_send, y2_send, switch_send, dial_send, cursor;
int receive, throttle;

void delay_ms(int x){
    int i;
    for(i = 0; i < x; i++) __delay_ms(1);
}

void delay_us(int x){
    int i;
    for(i = 0; i < x; i++) __delay_us(1);
}

void timer_init(unsigned char a){
    T0CS = 0;           //a = 7: 1:256
    T0SE = 0;           //a = 6: 1:128
    PSA = 0;            //a = 5: 1:64
    T0PS2 = (a / 4) % 2;//a = 4: 1:32
    T0PS1 = (a / 2) % 2;//a = 3: 1:16
    T0PS0 = a % 2;      //a = 2: 1:8
    IPEN = 0;           //a = 1: 1:4
    GIE = 1;            //a = 0: 1:2
    TMR0 = 0;
    TMR0IE = 1;
    TMR0ON = 1;
}

unsigned int get_adc(unsigned char port){
    ADCON0 = port * 4 + 1;
    ADCON0bits.GO_DONE = 1; 
    while(ADCON0bits.GO_DONE); 
    return (ADRESH * 256 + ADRESL); 
}

void get_adc_values(int *x1, int *y1, int *x2, int *y2, int *dial1, int *dial2){
    unsigned char i;
    for(*x1 = 0, i = 0; i < 10; i++){
        *x1 += get_adc(x1_pin);
        __delay_us(50);
    }
    for(*y1 = 0, i = 0; i < 10; i++){
        *y1 += get_adc(y1_pin);
        __delay_us(50);
    }
    for(*x2 = 0, i = 0; i < 10; i++){
        *x2 += get_adc(x2_pin);
        __delay_us(50);
    }
    for(*y2 = 0, i = 0; i < 10; i++){
        *y2 += get_adc(y2_pin);
        __delay_us(50);
    }
    for(*dial1 = 0, i = 0; i < 10; i++){
        *dial1 += get_adc(dial1_pin);
        __delay_us(50);
    }
    for(*dial2 = 0, i = 0; i < 10; i++){
        *dial2 += get_adc(dial2_pin);
        __delay_us(50);
    }
    *x1 /= 10;
    *y1 /= 10;
    *x2 /= 10;
    *y2 /= 10;
    *dial1 /= 10;
    *dial2 /= 10;
    *x1 = (*x1 - x1_offset) / 24;
    *y1 = (*y1 - y1_offset) / 20;
    *x2 = (*x2 - x2_offset) / 22;
    *y2 = (*y2 - y2_offset) / 19;
    *dial1 = (int)(float)(sqrt((float)*dial1) / 10.0);
    *dial2 = (int)(float)(sqrt((float)*dial2) / 10.0);
    if(*x1 > 15) *x1 = 15;
    else if(*x1 < (-15)) *x1 = (-15);
    if(*y1 > 15) *y1 = 15;
    else if(*y1 < (-15)) *y1 = (-15);
    if(*x2 > 15) *x2 = 15;
    else if(*x2 < (-15)) *x2 = (-15);
    if(*y2 > 31) *y2 = 31;
    if(*dial1 > 3) *dial1 = 3;
    if(*dial2 > 3) *dial2 = 3;
}

void interrupt ISR(){
    if(RCIF){
        receive = RCREG;
        if(OERR){
            CREN = 0;
            CREN = 1;
        }
    }
    if(TMR0IF){
        TMR0IF = 0;
        if(++counter == 4){
            USART_send(x1_send);
            __delay_us(100);
            USART_send(y1_send);
            __delay_us(100);
            USART_send(x2_send);
            __delay_us(100);
            USART_send(y2_send);
            __delay_us(100);
            USART_send(switch_send);
            __delay_us(100);
            USART_send(dial_send);
            counter = 0;
        }
    }
}

void init(){
    TRISA = 255;
    TRISB = 0;
    TRISC = 0;
    TRISD = 12;
    TRISE = 255;
    ADON = 1;
    ADCON1 = 9;
    ADCON2 = 185;
    UCONbits.USBEN = 0;
    UCFGbits.UTRDIS = 0;
}

void main(){
    int x1 = 0, x2 = 0, y1 = 0, y2 = 0, dial1 = 0, dial2 = 0;
    receive = 0;
    cursor = 0;
    init();
    USART_init(38400, 0);
    //SPI_init();
    //ColorLCD_init();
    led_red = 0;
    led_green = 1;
    led_blue = 1;
    delay_ms(100);
    throttle = 0;
    counter = 0;
    RCIE = 1;
    timer_init(7);
    while(1){
        get_adc_values(&x1, &y1, &x2, &y2, &dial1, &dial2);
        x1_send = (x1 + 15);
        y1_send = 32 + (y1 + 15);
        x2_send = 64 + (x2 + 15);
        y2_send = 96 + y2;
        switch_send = 128 + switch1 * 2 + switch2;
        dial_send = (160 + dial2 * 4 + dial1);
        delay_ms(13); 
    }
}

