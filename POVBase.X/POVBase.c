#define _XTAL_FREQ 20000000
#include <xc.h>
#include <pic16f876a.h>

#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config BOREN = OFF
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

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
    TRISB = 4;
    TRISC = 0;
}

void timer_init(){
    T0CS = 0;
    T0SE = 0;
    PSA = 0;
    PS2 = 1;
    PS1 = 1;
    PS0 = 1;
    GIE = 1;
    TMR0IE = 1; 
}

void interrupt ISR(){
    if(RCIF){
        receive = RCREG;
    }
    if(TMR0IF){
        TMR0IF = 0;
    }
}

void main(){    
    int i;
    init();
    __delay_ms(100);
    for(i = 0; i < 245; i++){
        RC0 = 1;
        __delay_ms(1);
        RC0 = 0;
        __delay_ms(1);
        __delay_us(40);
    }
    for(i = 0; i < 2205; i++){
        RC0 = 1;
        __delay_ms(2);
        RC0 = 0;
        __delay_us(40);
    }
    for(i = 0; i < 1000; i++){
        RC0 = 1;
        __delay_ms(1);
        RC0 = 0;
        __delay_ms(1);
        __delay_us(40);
    }
    while(1){
        RC0 = 1;
        __delay_ms(1);
        __delay_us(50);
        RC0 = 0;
        __delay_us(950);
        __delay_us(40);
    }
}

