#define _XTAL_FREQ 48000000
#include <xc.h>
#include <pic18f4550.h>

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
    TRISD = 0;
    TRISE = 0;
}

void timer_init(){
    T0CS = 0;
    T0SE = 0;
    PSA = 0;
    T0PS2 = 0;
    T0PS1 = 1;
    T0PS0 = 1;
    IPEN = 0;
    GIE = 1;
    TMR0 = 181;
    TMR0IE = 1;
    TMR0ON = 1;
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
    while(1){
        PORTDbits.RD2 = 0;
        PORTDbits.RD3 = 1;
        __delay_us(100);
        PORTDbits.RD2 = 1;
        PORTDbits.RD3 = 0;
        __delay_us(100);
    }
}

