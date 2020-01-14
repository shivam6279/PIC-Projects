#include "pic32.h"
#include <xc.h>

unsigned long int delay_counter = 0;

void adc_init(){
    ADCCON1bits.ON = 0; 
    
    ADCCON1 = 0;
    ADCCON1bits.STRGSRC = 1;
    
    ADCCON2bits.SAMC = 1023;
    ADCCON2bits.ADCDIV = 4;
    
    ADCCON3 = 0;
    ADCCON3bits.ADCSEL = 0;
    ADCCON3bits.CONCLKDIV = 1;
    ADCCON3bits.VREFSEL = 0;
    
    ADCIMCON1 = 0;
    ADCIMCON2 = 0;
    ADCIMCON3 = 0;
    
    ADCGIRQEN1 = 0;
    ADCGIRQEN2 = 0;
    
    ADCCMPCON1 = 0;                    
    ADCCMPCON2 = 0;
    ADCCMPCON3 = 0;
    ADCCMPCON4 = 0;
    ADCCMPCON5 = 0;
    ADCCMPCON6 = 0;
    
    ADCFLTR1 = 0;                     
    ADCFLTR2 = 0;
    ADCFLTR3 = 0;
    ADCFLTR4 = 0;
    ADCFLTR5 = 0;
    ADCFLTR6 = 0;
    
    ADCEIEN1 = 0; 
    ADCEIEN2 = 0;
    
    ADCTRGSNS = 0;
    
    ADCTRG1 = 0;                      
    ADCTRG2 = 0;                   
    ADCTRG3 = 0;
    
    ADCTRGMODEbits.SH4ALT = 1;
    ADCTRGMODEbits.SH3ALT = 1;
    ADCTRG1bits.TRGSRC3 = 1;
    ADCTRG2bits.TRGSRC4 = 1;
    
    ADCTRG3bits.TRGSRC9 = 1;
    ADCTRG3bits.TRGSRC10 = 1;
    ADCTRG2bits.TRGSRC7 = 1;
    
    ADCCSS1 = 0;
    ADCCSS2 = 0;
    
    ADCCSS1bits.CSS3 = 1;
    ADCCSS1bits.CSS4 = 1;
    
    ADCCSS1bits.CSS9 = 1;
    ADCCSS1bits.CSS10 = 1;
    ADCCSS1bits.CSS12 = 1;
    ADCCSS1bits.CSS13 = 1;
    ADCCSS1bits.CSS15 = 1;
    ADCCSS1bits.CSS16 = 1;
    ADCCSS1bits.CSS17 = 1;
    ADCCSS1bits.CSS18 = 1;
    
    ADCANCONbits.WKUPCLKCNT = 5;
    
    ADCCON1bits.ON = 1; 
    
    while(!ADCCON2bits.BGVRRDY);
    while(ADCCON2bits.REFFLT); 
    
    ADCANCONbits.ANEN7 = 1;
    ADCCON3bits.DIGEN7 = 1;
    
    ADCANCONbits.ANEN3 = 1;
    ADCANCONbits.ANEN4 = 1;
    ADCCON3bits.DIGEN3 = 1;
    ADCCON3bits.DIGEN4 = 1;
}

void init(){
    //IO pins
    TRISB = 0xE040;
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 1;
    
    TRISC = 0;
    TRISD = 0;
    TRISE = 0xF0;
    TRISF = 0;
    TRISG = 0x0180;
    ANSELB = 0xC000;
    ANSELE = 0xF0;
    ANSELG = 0x0180;
    
    ANSELBbits.ANSB8 = 1;
    ANSELBbits.ANSB9 = 1;
    
    PRECONbits.PREFEN = 3;
    PRECONbits.PFMWS = 2;
    SYSKEY = 0xAA996655;//Unlocking
    SYSKEY = 0x556699AA;//Sequence
    OSCCONbits.FRCDIV = 0;
    OSCCONbits.COSC = 1;
    OSCTUNbits.TUN = 0;
    //SYSKEY = 0x33333333;//Locking sequence
    
    PRISS = 0x76543210;
    INTCONbits.MVEC = 1;
    
    PB2DIVbits.ON = 1;
    PB2DIVbits.PBDIV = 1;//PBCLK2 at 100mhz
    
    PB3DIVbits.ON = 1;
    PB3DIVbits.PBDIV = 1;//PBCLK3 at 100mhz
    
    __asm__("ei");//Enable interrupts
}

void delay_ms(unsigned int x){
    delay_counter = 0;
    T2CONbits.TON = 1;
    while(delay_counter < x);
    T2CONbits.TON = 0;
}

void timer2_init(float frequency) {
    float f = 100000000.0 / frequency; 
    unsigned char pre = 0;
    while(f > 65535.0) { 
        f /= 2.0;
        pre++; 
    }
    unsigned int t = (unsigned int)f;
    while(t % 2 == 0 && pre < 8) { 
        t /= 2; 
        pre++; 
    }
    if(pre == 7) {
        if(t > 32767) {
            t /= 2;
            pre++;
        } else {
            t *= 2; 
            pre--;
        }
    }
    if(pre == 8) pre = 7;
    T2CONbits.ON = 0;
    T2CONbits.T32 = 0;
    T2CONbits.TCKPS = pre & 0b111;
    PR2 = t;
    TMR2 = 0;
    
    IPC2bits.T2IP = 4;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
}

void timer3_init(float frequency){
    float t = 100000000.0 / frequency; unsigned char pre = 0;
    while(t > 65535){ t /= 2.0; pre++; }
    t = (int)t;
    while((int)t % 2 == 0 && pre < 8){ t /= 2.0; pre++; }
    if(pre == 7){ t *= 2.0; pre--; }
    if(pre == 8) pre = 7;
    T3CONbits.TON = 0;
    T3CONbits.TCKPS = pre;//50hz
    PR3 = (int)t - 1;
    TMR3 = 0;
    IPC3bits.T3IP = 3;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    T3CONbits.TON = 1;
}

void timer4_init(float frequency){
    float t = 100000000.0 / frequency; unsigned char pre = 0;
    while(t > 65535){ t /= 2.0; pre++; }
    t = (int)t;
    while((int)t % 2 == 0 && pre < 8){ t /= 2.0; pre++; }
    if(pre == 7){ t *= 2.0; pre--; }
    if(pre == 8) pre = 7;
    T4CONbits.ON = 0;
    T4CONbits.TCKPS = pre;
    PR4 = (int)t - 1;
    TMR4 = 0;
    IPC4bits.T4IP = 7;
    IFS0bits.T4IF = 0;
    IEC0bits.T4IE = 1;
    T4CONbits.TON = 1;
}

void timer5_init(float frequency){
    float t = 100000000.0 / frequency; unsigned char pre = 0;
    while(t > 65535){ t /= 2.0; pre++; }
    t = (int)t;
    while((int)t % 2 == 0 && pre < 8){ t /= 2.0; pre++; }
    if(pre == 7){ t *= 2.0; pre--; }
    if(pre == 8) pre = 7;
    T5CONbits.ON = 0;
    T5CONbits.TCKPS = pre;
    PR5 = (int)t - 1;
    TMR5 = 0;
    IPC6bits.T5IP = 4;
    IFS0bits.T5IF = 0;
    IEC0bits.T5IE = 1;
    T5CONbits.ON = 1;
}