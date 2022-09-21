#include "PWM.h"
#include "bitbang_I2C.h"
#include "motor.h"
#include "settings.h"
#include <xc.h>
#include <sys/attribs.h>

unsigned int PWM_MAX;
unsigned int MOTOR_OFF, MOTOR_MAX;

void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL4SRS) pwm(void){
    IFS0bits.T3IF = 0;
}

void pwm_init(float freq) {
    float f = 100000000.0 / freq; 
    unsigned char pre = 0;
    while(f > 65535.0) { 
        f /= 2.0;
        pre++; 
    }
    unsigned int t = (unsigned int)f;
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

    float temp;
    PWM_MAX = t;
    temp = MOTOR_OFF_TIME / 1000000.0 * freq * (float)PWM_MAX;
    MOTOR_OFF = (int)temp;
    temp = MOTOR_MAX_TIME / 1000000.0 * freq * (float)PWM_MAX;
    MOTOR_MAX = (int)temp;

    CFGCONbits.OCACLK = 0;

    OC1CON = 0;  
    OC1R = 0;
    OC1RS = 0;
    OC1CON = 0b1110;   

    OC2CON = 0;  
    OC2R = 0;
    OC2RS = 0;
    OC2CON = 0b1110;   

    OC3CON = 0;  
    OC3R = 0;
    OC3RS = 0;
    OC3CON = 0b1110;   

    OC4CON = 0;  
    OC4R = 0;
    OC4RS = 0;
    OC4CON = 0b1110;   

    OC5CON = 0;  
    OC5R = 0;
    OC5RS = 0;
    OC5CON = 0b1110;   

    OC6CON = 0;  
    OC6R = 0;
    OC6RS = 0;
    OC6CON = 0b1110;   

    OC8CON = 0;  
    OC8R = 0;
    OC8RS = 0;
    OC8CON = 0b1110;

    T3CONbits.TCKPS = pre & 0b111;
    PR3 = PWM_MAX;
    IPC3bits.T3IP = 4;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;

    OC1CONbits.ON = 1;
    OC2CONbits.ON = 1;
    OC3CONbits.ON = 1;
    OC4CONbits.ON = 1;
    OC5CONbits.ON = 1;
    OC6CONbits.ON = 1;
    OC8CONbits.ON = 1;

    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB9 = 0;
    TRISBbits.TRISB10 = 0;
    TRISGbits.TRISG7 = 0;
    TRISGbits.TRISG9 = 0;

    CFGCONbits.IOLOCK = 0; // Unlock IO

    RPB6Rbits.RPB6R = 0b1100;   //OC1
    RPG9Rbits.RPG9R = 0b1011;   //OC2
    RPB9Rbits.RPB9R = 0b1011;   //OC3
    RPG7Rbits.RPG7R = 0b1011;   //OC4
    RPB8Rbits.RPB8R = 0b1011;   //OC5
    RPB10Rbits.RPB10R = 0b1100; //OC6        
    RPB7Rbits.RPB7R = 0b1100;   //OC8

    CFGCONbits.IOLOCK = 1;  // Lock IO       

    T3CONbits.TON = 1;
}

void write_pwm(int num, int val){
    if (val <= 0) {
        val = 0;
    }
    else if (val >= PWM_MAX) {
        val = PWM_MAX;
    }
    switch(num) {
        case 1:
            OC1RS = val;
            break;                
        case 2:
            OC2RS = val;
            break;                
        case 3:
            OC3RS = val;
            break;                
        case 4:
            OC4RS = val;
            break;                
        case 5:
            OC5RS = val;
            break;                
        case 6:
            OC6RS = val;
            break;                
        case 7:
            OC7RS = val;
            break;                
        case 8:
            OC8RS = val;
            break;                
        case 9:
            OC9RS = val;
            break;
    }
}