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

#define reed_pin PORTBbits.RB1

unsigned char receive, reed_flag = 1, timer_counter = 0;
unsigned long int speed_counter = 0;
float speed;

void pwm_init(){
    CCP2CON = 12;
    //CCP2CON = 60;
    PR2 = 74;
    T2CON = 7;
    CCPR2L = 0;
}

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
    PSA = 1;
    PS2 = 0;
    PS1 = 0;
    PS0 = 0;
    //TMR0 = 156;
    GIE = 1;
    TMR0IE = 1; 
}

void timer1_init(){
    T1CKPS0 = 0;
    T1CKPS1 = 0;
    T1OSCEN = 0;
    TMR1CS = 0;
    TMR1ON = 1;
    TMR1L = 0x63;
    TMR1H = 0xFF;
    GIE = 1;
    TMR1IE = 1; 
}

void interrupt ISR(){
    if(TMR0IF){
        TMR0IF = 0;
        RC2 = !RC2;
        RC3 = !RC3;
        timer_counter++;
        //TMR0 = 156;
    }
    if(TMR1IF){
        TMR1IF = 0;
        speed_counter++;
        if(reed_pin == 0 && reed_flag == 1){
            reed_flag = 0;
            speed = 60 / (float)speed_counter * 4006.41;
            speed_counter = 0;
        }
        else if(reed_pin == 1){
            reed_flag = 1;
        }
        TMR1L = 0x63;
        TMR1H = 0xFF;
    }
}

inline void one() {
    RC0 = 0;
    RC3 = 1;
    __delay_us(12);
    RC3 = 0;
    __delay_us(5);
}

inline void zero() {
    RC2 = 1;
    RC3 = 0;
    __delay_us(10);
    RC2 = 0;
    __delay_us(6);
}

void main(){    
    float output, t_speed = 0.0, p_speed, sum = 0.0;
    unsigned int pwm;
    int i;
    init();
    RC2 = 0;
    RC3 = 0;
    delay_ms(10);
    //timer_init();
    //timer1_init();
    pwm_init();
    pwm = 200;//270
    CCP2CONbits.CCP2X = (pwm >> 1) & 1;
    CCP2CONbits.CCP2Y = pwm & 1;
    CCPR2L = pwm >> 2;
    
    while(1){
        RC3 = 1;
        __delay_us(12);
        RC3 = 0;
        __delay_us(5);
        RC2 = 1;
        __delay_us(10);
        RC2 = 0;
        __delay_us(6);
    }
    while(1){
        one();
        zero();
        zero();
        one();
        one();
        zero();
        zero();
        one();
        one();
        zero();
        zero();
        one();
        one();
        zero();
        zero();
        one();
    }
    while(1){
        timer_counter = 0;
        p_speed = t_speed;
        t_speed = speed;
        sum += 0.005* (t_speed - 1000.0);
        output = 1.0 * (t_speed - 1000.0) + 1.0 * sum + 1.0 * (t_speed - p_speed) / 0.005;
        if(output < 0.0) pwm = 0;
        else if(output > 1023.0) pwm = 1023;
        else pwm = (unsigned int)output;
        CCP2CONbits.CCP2X = (pwm >> 1) & 1;
        CCP2CONbits.CCP2Y = pwm & 1;
        CCPR2L = pwm >> 2;
        while(timer_counter < 98);
    }
}

