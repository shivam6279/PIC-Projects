#include "PWM.h"
#include "bitbang_I2C.h"
#include "motor.h"
#include "settings.h"
#include <xc.h>

int pwm_max;
int motor_off, motor_max;

#ifdef board_v4
    void pwm_init(float freq) {
        float f = 100000000.0 / freq; 
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

        pwm_max = t;
        t = MOTOR_OFF / 1000000.0 * freq * (float)pwm_max;
        motor_off = (int)t;

        OC1CON = 0;  
        OC1R = motor_off;
        OC1RS = motor_off;
        OC1CON = 0b1100;   

        OC2CON = 0;  
        OC2R = motor_off;
        OC2RS = motor_off;
        OC2CON = 0b1100;   

        OC3CON = 0;  
        OC3R = motor_off;
        OC3RS = motor_off;
        OC3CON = 0b1100;   

        OC4CON = 0;  
        OC4R = motor_off;
        OC4RS = motor_off;
        OC4CON = 0b1100;   

        OC5CON = 0;  
        OC5R = motor_off;
        OC5RS = motor_off;
        OC5CON = 0b1100;   

        OC8CON = 0;  
        OC8R = motor_off;
        OC8RS = motor_off;
        OC8CON = 0b1100;   

        OC9CON = 0;  
        OC9R = motor_off;
        OC9RS = motor_off;
        OC9CON = 0b1100;

        T3CONbits.TCKPS = pre & 0b111;
        PR3 = pwm_max;
        T3CONbits.TON   = 1;
        
        OC1CONbits.ON = 1;
        OC2CONbits.ON = 1;
        OC3CONbits.ON = 1;
        OC4CONbits.ON = 1;
        OC5CONbits.ON = 1;
        OC8CONbits.ON = 1;
        OC9CONbits.ON = 1;  

        
        TRISBbits.TRISB2 = 0;
        TRISBbits.TRISB3 = 0;
        TRISBbits.TRISB5 = 0;
        TRISBbits.TRISB7 = 0;
        TRISBbits.TRISB14 = 0;
        TRISBbits.TRISB15 = 0;
        TRISGbits.TRISG9 = 0;

        CFGCONbits.IOLOCK = 0; // Unlock IO

        RPB5Rbits.RPB5R = 0b1011;    //OC3
        RPB3Rbits.RPB3R = 0b1011;    //OC4
        RPB15Rbits.RPB15R = 0b1011;  //OC5
        RPB7Rbits.RPB7R = 0b1100;    //OC8
        RPB2Rbits.RPB2R = 0b1011;    //OC2
        RPG9Rbits.RPG9R = 0b1100;    //OC1        
        RPB14Rbits.RPB14R = 0b1101;  //OC9

        CFGCONbits.IOLOCK = 1;  // Lock IO       
    }
    void write_pwm(int num, int val){
        if (val <= 0) {
            val = 0;
        }
        else if (val >= 4095) {
            val = 4095;
        }
        if(num == 1) {
            OC1R = val;
        } else if(num == 2) {
            OC2R = val;
        } else if(num == 3) {
            OC3R = val;
        } else if(num == 4) { 
            OC4R = val;
        } else if(num == 5) {
            OC5R = val;
        } else if(num == 6) {
            OC6R = val;
        } else if(num == 7) {
            OC7R = val;
        } else if(num == 8) {
            OC8R = val;
        } else if(num == 9) {
            OC9R = val;
        }  
    }

#else
    void pwm_init(float freq){
        //Highest frequency 1600hz
        unsigned int t = 20000;
        unsigned char prescale;
        float prescaleval;
        freq *= 0.9;
        prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        prescale = (int)(prescaleval + 0.5);

        pwm_max = 4097;
        prescaleval = MOTOR_OFF / 1000000.0 * freq * (float)pwm_max;
        motor_off = (int)prescaleval;

        I2C_WriteRegisters(128, (unsigned char[2]){0, 16}, 2);
        I2C_WriteRegisters(128, (unsigned char[2]){0xFE, prescale}, 2);
        I2C_WriteRegisters(128, (unsigned char[2]){0, 0}, 2);
        while(t-- > 0);
        I2C_WriteRegisters(128, (unsigned char[2]){0, 0xA1}, 2);
    }

    void set_pwm(int num, int on, int off){
        I2C_WriteRegisters(128, (unsigned char[5]){(0x06 + 4 * num), (on & 0xFF), ((on >> 8) & 0x1F), (off & 0xFF), ((off >> 8) & 0x1F)}, 5);
    }

    void write_pwm(int num, int val){
        if (val == 0) {
            set_pwm(num, 0, 4096);
        }
        else if (val >= 4095) {
            set_pwm(num, 4096, 0);
        } else {
            set_pwm(num, 0, val);
        }
    }
#endif