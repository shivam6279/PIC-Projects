#include "pic32.h"
#include <sys/attribs.h>
#include <xc.h>

static volatile unsigned long int delay_counter = 0;

void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4AUTO) delay_timer(void){
    IFS0bits.T2IF = 0;
    delay_counter++;
}

void PICInit(){    
    TRISB = 0xFFFFFFFF; 
    TRISC = 0xFFFFFFFF; 
    TRISD = 0xFFFFFFFF; 
    TRISE = 0xFFFFFFFF; 
    TRISF = 0xFFFFFFFF;
    TRISG = 0xFFFFFFFF;
    
    ANSELB = 0;
    ANSELE = 0;
    ANSELG = 0;
    
    I2C1CONbits.ON = 0;
    I2C3CONbits.ON = 0;
    I2C4CONbits.ON = 0;
    I2C5CONbits.ON = 0;
    
    ODCB = 0;
    ODCC = 0;
    ODCD = 0;
    ODCF = 0;
    ODCG = 0;
    
    CNPUB = 0;
    CNPUC = 0;
    CNPUD = 0;
    CNPUE = 0;
    CNPUF = 0;
    CNPUG = 0;
    
    CNPDB = 0;
    CNPDC = 0;
    CNPDD = 0;
    CNPDE = 0;
    CNPDF = 0;
    CNPDG = 0;
    
    PMCON = 0;
    PMAEN = 0;
    
    CM1CONbits.ON = 0;
    CM2CONbits.ON = 0;
    
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

void StartDelayCounter() {
    delay_counter = 0;
    DELAY_TIMER_ON = 1;
}

void StopDelayCounter() {
    DELAY_TIMER_ON = 0;
}

unsigned long int ms_counter() {
    return delay_counter;
}

void delay_ms(unsigned int x){
    StartDelayCounter();
    while(delay_counter < x);
    StopDelayCounter();
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
    T3CONbits.ON = 0;
    T3CONbits.TCKPS = pre & 0b111;
    PR3 = t;
    TMR3 = 0;
    IPC3bits.T3IP = 4;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
}

void timer4_init(float frequency){
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
    T4CONbits.ON = 0;
    T4CONbits.T32 = 0;
    T4CONbits.TCKPS = pre & 0b111;
    PR4 = t;
    TMR4 = 0;
    
    IPC4bits.T4IP = 7;
    IFS0bits.T4IF = 0;
    IEC0bits.T4IE = 1;
}

void timer5_init(float frequency){
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
    T5CONbits.ON = 0;
    T5CONbits.TCKPS = pre & 0b111;
    PR5 = t;
    TMR5 = 0;
    IPC6bits.T5IP = 4;
    IFS0bits.T5IF = 0;
    IEC0bits.T5IE = 1;
}

void timer6_init(float frequency){
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
    T6CONbits.ON = 0;
    T6CONbits.T32 = 0;
    T6CONbits.TCKPS = pre & 0b111;
    PR6 = t;
    TMR6 = 0;
    IPC7bits.T6IP = 4;
    IFS0bits.T6IF = 0;
    IEC0bits.T6IE = 1;
}

void timer7_init(float frequency){
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
    T7CONbits.ON = 0;
    T7CONbits.TCKPS = pre & 0b111;
    PR7 = t;
    TMR7 = 0;
    IPC8bits.T7IP = 4;
    IFS1bits.T7IF = 0;
    IEC1bits.T7IE = 1;
}