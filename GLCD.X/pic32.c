#include "pic32.h"
#include <sys/attribs.h>
#include <xc.h>

volatile unsigned long int delay_counter = 0;

void PICInit(){        
    // Unlocking sequence
    SYSKEY = 0x00000000U;
    SYSKEY = 0xAA996655U;
    SYSKEY = 0x556699AAU;
    
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
    
    PRECONbits.PREFEN = 3;
    PRECONbits.PFMWS = 2;
    CFGCONbits.ECCCON = 3;
    
    __asm__("ei");//Enable interrupts
    
    //IO pins    
    TRISB = 0xFFFF; 
    TRISC = 0xFFFF; 
    TRISD = 0xFFFF; 
    TRISE = 0xFFFF; 
    TRISF = 0xFFFF;
    TRISG = 0xFFFF;
    TRISH = 0xFFFF;
    TRISK = 0xFFFF;
    
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELE = 0;
    ANSELF = 0;
    ANSELG = 0;
    ANSELH = 0;
    
    SRCON0A = 0;
    SRCON1A = 0;
    
    SRCON0B = 0;
    SRCON1B = 0;
    
    SRCON0C = 0;
    SRCON1C = 0;
    
    SRCON0D = 0;
    SRCON1D = 0;
    
    SRCON0E = 0;
    SRCON1E = 0;
    
    SRCON0F = 0;
    SRCON1F = 0;
    
    SRCON0G = 0;
    SRCON1G = 0;
    
    SRCON0H = 0;
    SRCON1H = 0;
    
    SRCON0J = 0;
    SRCON1J = 0;
    
    SRCON0K = 0;
    SRCON1K = 0;
}

void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4SRS) delay_timer(void){
    IFS0bits.T2IF = 0;
    delay_counter++;
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