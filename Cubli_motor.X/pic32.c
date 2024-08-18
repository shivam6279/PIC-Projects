#include "pic32.h"
#include <sys/attribs.h>
#include <xc.h>

static volatile unsigned long int delay_ms_counter1 = 0, delay_ms_counter2 = 0, delay_ms_counter3 = 0;
static volatile unsigned long int delay_us_counter = 0;

void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4AUTO) delay_ms_timer(void){
    IFS0bits.T2IF = 0;
    delay_ms_counter1++;
    delay_ms_counter2++;
    delay_ms_counter3++;
}

void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL7AUTO) delay_us_timer(void){
    IFS0bits.T3IF = 0;
    delay_us_counter++;
}

void PICInit(){
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ANSELE = 0;
    ANSELG = 0;
    
    CNPUA = 0;
    CNPUB = 0;
    CNPUC = 0;
    CNPUD = 0;
    CNPUE = 0;
    CNPUF = 0;
    CNPUG = 0;
    
    CNPDA = 0;
    CNPDB = 0;
    CNPDC = 0;
    CNPDD = 0;
    CNPDE = 0;
    CNPDF = 0;
    CNPDG = 0;
    
    CNCONA = 0;
    CNCONB = 0;
    CNCONC = 0;
    CNCOND = 0;
    CNCONE = 0;
    CNCONF = 0;
    CNCONG = 0;
    
    TRISB = 0xFFFFFFFF; 
    TRISC = 0xFFFFFFFF; 
    TRISD = 0xFFFFFFFF; 
    TRISE = 0xFFFFFFFF; 
    TRISF = 0xFFFFFFFF;
    
    TRISAbits.TRISA8 = 0;
    TRISDbits.TRISD6 = 0;
    
    CM1CON = 0;
    CM2CON = 0;
    CM3CON = 0;
    CM4CON = 0;
    CM5CON = 0;
    
    DAC1CON = 0;
    DAC2CON = 0;
    DAC3CON = 0;
    
    PMD6bits.PMPMD = 1;
    PMCON = 0;
    PMAEN = 0;
    
    SYSKEY = 0xAA996655;//Unlocking
    SYSKEY = 0x556699AA;//Sequence
    OSCCONbits.FRCDIV = 0;
    OSCCONbits.COSC = 1;
    OSCCONbits.SOSCEN = 0;
//    OSCTUNbits.TUN = 0;
    //SYSKEY = 0x33333333;//Locking sequence
    
    PRISS = 0x76543210;
    INTCONbits.MVEC = 1;
    
    PB2DIVbits.ON = 1;
    PB2DIVbits.PBDIV = 1;//PBCLK2 at 60mhz
    
    PB3DIVbits.ON = 1;
    PB3DIVbits.PBDIV = 1;//PBCLK3 at 60mhz
    
    PB4DIVbits.ON = 1;
    PB4DIVbits.PBDIV = 1;//PBCLK3 at 60mhz
    
    PB5DIVbits.ON = 1;
    PB5DIVbits.PBDIV = 1;//PBCLK5 at 60mhz
    
    PB6DIVbits.ON = 1;
    PB6DIVbits.PBDIV = 1;//PBCLK6 at 60mhz
    
    __builtin_enable_interrupts();
}

void ChangeNotificationInit() {
    CNCONB = 0;
    CNENBbits.CNIEB5 = 1;
    CNENBbits.CNIEB6 = 1;
    CNENBbits.CNIEB7 = 1;       
    IFS1bits.CNBIF = 0;
    IPC11bits.CNBIP = 5;
    IPC11bits.CNBIS = 0;
    IEC1bits.CNBIE = 1;    
    CNCONBbits.ON = 1;   
}

void QEI_init() {
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    TRISCbits.TRISC8 = 1;
    TRISBbits.TRISB6 = 1;
    CFGCONbits.IOLOCK = 0;
    QEA1Rbits.QEA1R = 0b0101;  // QEA1 @ RC7
    QEB1Rbits.QEB1R = 0b0110;  // QEB1 @ RC8
    INDX1Rbits.INDX1R = 0b000; // INDX1 @ RPB6
    CFGCONbits.IOLOCK = 1;
    QEI1CON = 0;    
    QEI1IOC = 0;    
    QEI1CONbits.PIMOD = 0b001;
    QEI1CONbits.QEIEN = 1;
}

void StartDelaymsCounter() {
    delay_ms_counter1 = 0;
    delay_ms_counter2 = 0;
    delay_ms_counter3 = 0;
    T2CONbits.ON = 1;
}

void StopDelaymsCounter() {
    T2CONbits.ON = 0;
}

unsigned long int ms_counter() {
    return delay_ms_counter1;
}

void reset_ms_counter() {
    delay_ms_counter1 = 0;
}

unsigned long int ms_counter2() {
    return delay_ms_counter2;
}

void reset_ms_counter2() {
    delay_ms_counter2 = 0;
}

unsigned long int ms_counter3() {
    return delay_ms_counter3;
}

void reset_ms_counter3() {
    delay_ms_counter3 = 0;
}

void delay_ms(unsigned int x){
    StartDelaymsCounter();
    while(delay_ms_counter1 < x);
    StopDelaymsCounter();
}

void StartDelayusCounter() {
    delay_us_counter = 0;
    T3CONbits.ON = 1;
}

void StopDelayusCounter() {
    T3CONbits.ON = 0;
}

unsigned long int us_counter() {
    return delay_us_counter;
}

void reset_us_counter() {
    delay_us_counter = 0;
}

void delay_us(unsigned int x){
    StartDelayusCounter();
    while(delay_us_counter < x);
    StopDelayusCounter();
}

void timer2_init(float frequency) {
    float f = 60000000.0 / frequency; 
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
    T2CONbits.TCS = 0;
    PR2 = t;
    TMR2 = 0;
    
    IPC2bits.T2IP = 4;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
}

void timer3_init(float frequency){
    float f = 60000000.0 / frequency; 
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
    T3CONbits.T32 = 0;
    T3CONbits.TCKPS = pre & 0b111;
    T3CONbits.TCS = 0;
    PR3 = t;
    TMR3 = 0;
    
    IPC3bits.T3IP = 7;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
}

void timer4_init(float frequency){
    float f = 60000000.0 / frequency; 
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
    T4CONbits.TCS = 0;
    PR4 = t;
    TMR4 = 0;
    
    IPC4bits.T4IP = 6;
    IFS0bits.T4IF = 0;
    IEC0bits.T4IE = 1;
}

void timer5_init(float frequency){
    float f = 60000000.0 / frequency; 
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
    IPC6bits.T5IP = 3;
    IFS0bits.T5IF = 0;
    IEC0bits.T5IE = 1;
}

void timer6_init(float frequency){
    float f = 60000000.0 / frequency; 
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
    T6CONbits.ON = 1;
    T6CONbits.T32 = 0;
    T6CONbits.TCKPS = pre & 0b111;
    PR6 = t;
    TMR6 = 0;
    IPC19bits.T6IP = 4;
    IFS2bits.T6IF = 0;
    IEC2bits.T6IE = 1;
}

void timer7_init(float frequency){
    float f = 60000000.0 / frequency; 
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
    IPC20bits.T7IP = 4;
    IFS2bits.T7IF = 0;
    IEC2bits.T7IE = 1;
}