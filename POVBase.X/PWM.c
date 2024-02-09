#include "PWM.h"
#include <xc.h>
#include <sys/attribs.h>

#define DEAD_TIME 5

unsigned int PWM_MAX;

void PwmInit() {
    float freq1 = 24000;
    float freq2 = 145000;    
    //118000
    float temp = 120000000.0 / 50000;
    PWM_MAX = temp;
    
    TRISBbits.TRISB10 = 0;
    TRISBbits.TRISB11 = 0;
    TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB13 = 0;
    TRISBbits.TRISB14 = 0;
    TRISBbits.TRISB15 = 0;
    TRISFbits.TRISF0 = 0;
    TRISFbits.TRISF1 = 0;
    TRISDbits.TRISD5 = 0;
    TRISDbits.TRISD6 = 0;

    PTCON = 0;
    PTCONbits.PTEN = 0;

    STCON = 0;
    CHOP = 0;

    PTCONbits.PCLKDIV = 0;

    CFGCONbits.PWMAPIN1 = 1;
    CFGCONbits.PWMAPIN2 = 1;
    CFGCONbits.PWMAPIN3 = 1;

    PMD4bits.PWM1MD = 0;
    PMD4bits.PWM2MD = 0;
    PMD4bits.PWM3MD = 0;
    PMD4bits.PWM4MD = 1;
    PMD4bits.PWM5MD = 1;
    PMD4bits.PWM6MD = 1;
    PMD4bits.PWM7MD = 0;
    PMD4bits.PWM8MD = 0;
    PMD4bits.PWM9MD = 0;
    PMD4bits.PWM10MD = 1;
    PMD4bits.PWM11MD = 0;
    PMD4bits.PWM12MD = 0;
    
    IOCON1 = 0;
    IOCON2 = 0;
    IOCON3 = 0;
    IOCON4 = 0;
    IOCON5 = 0;
    IOCON6 = 0;
    IOCON7 = 0;
    IOCON8 = 0;
    IOCON9 = 0;
    IOCON10 = 0;
    IOCON11 = 0;
    IOCON12 = 0;
    
    IOCON1bits.FLTMOD = 0b11;
    IOCON2bits.FLTMOD = 0b11;
    IOCON3bits.FLTMOD = 0b11;
    IOCON4bits.FLTMOD = 0b11;
    IOCON5bits.FLTMOD = 0b11;
    IOCON6bits.FLTMOD = 0b11;
    IOCON7bits.FLTMOD = 0b11;
    IOCON8bits.FLTMOD = 0b11;
    IOCON9bits.FLTMOD = 0b11;
    IOCON10bits.FLTMOD = 0b11;
    IOCON11bits.FLTMOD = 0b11;
    IOCON12bits.FLTMOD = 0b11;

    PWMCON1 = 0;
    PWMCON2 = 0;
    PWMCON3 = 0;
    
    PWMCON1bits.ECAM = 0;
    PWMCON2bits.ECAM = 0;
    PWMCON3bits.ECAM = 0;   
    PWMCON12bits.ECAM = 0;
    
    IOCON1bits.PENH = 1;
    IOCON1bits.SWAP = 1;
    PDC1 = 0;
    DTR1 = DEAD_TIME;
    ALTDTR1 = DEAD_TIME;

    IOCON2bits.PENH = 1;
    IOCON2bits.SWAP = 1;
    PDC2 = 0;
    DTR2 = DEAD_TIME;
    ALTDTR2 = DEAD_TIME;

    IOCON3bits.PENH = 1;
    IOCON3bits.SWAP = 1;
    PDC3 = 0;
    DTR3 = DEAD_TIME;
    ALTDTR3 = DEAD_TIME;
    
    IOCON7bits.PENH = 1;
    PDC7 = 0;
    DTR7 = DEAD_TIME;
    ALTDTR7 = DEAD_TIME;
    
    IOCON8bits.PENH = 1;
    PDC8 = 0;
    DTR8 = DEAD_TIME;
    ALTDTR8 = DEAD_TIME;
    
    IOCON9bits.PENH = 1;
    PDC9 = 0;
    DTR9 = DEAD_TIME;
    ALTDTR9 = DEAD_TIME;  
    
    unsigned long COIL_DTR = 20;
    
    IOCON11bits.PENH = 1;
    IOCON11bits.PENL = 1;
    IOCON11bits.SWAP = 1;
    PDC11 = 0;
    DTR11 = COIL_DTR;
    ALTDTR11 = COIL_DTR;  
    
    IOCON12bits.PENH = 1;
    IOCON12bits.PENL = 1;
    IOCON12bits.SWAP = 0;
    PDC12 = PWM_MAX;
    DTR12 = COIL_DTR;
    ALTDTR12 = COIL_DTR;  
    
    PTPERbits.PTPER = PWM_MAX;

    PTCONbits.PTEN = 1;
}

void WritePwm(int num, int val){
    if (val <= 0) {
        val = 0;
    }
    else if (val >= PWM_MAX) {
        val = PWM_MAX;
    }
    switch(num) {
        case 1:
            PDC1 = val;
            break;                
        case 2:
            PDC2 = val;
            break;                
        case 3:
            PDC3 = val;
            break;                
        case 4:
            PDC4 = val;
            break;                
        case 5:
            PDC5 = val;
            break;                
        case 6:
            PDC6 = val;
            break;
        case 7:
            PDC7 = val;
            break;                
        case 8:
            PDC8 = val;
            break;                
        case 9:
            PDC9 = val;
            break;                
        case 10:
            PDC10 = val;
            break;                
        case 11:
            PDC11 = val;
            break;                
        case 12:
            PDC12 = val;
            break;
    }
}