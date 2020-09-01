#include "BLDC.h"
#include "PWM.h"
#include <xc.h>
#include <stdbool.h>
#include <sys/attribs.h>
#include "USART.h"

volatile unsigned char current_phase = 0;
volatile signed char commutate = 0;
volatile unsigned int motor_speed = 0;

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL3AUTO) BLDC_timing(void){
    IFS0bits.T4IF = 0;    
    if(commutate) {
        MotorPhase(current_phase + commutate, motor_speed);
    }    
    T4CONbits.ON = 0;
}

void __ISR_AT_VECTOR(_CHANGE_NOTICE_B_VECTOR, IPL5AUTO) HallPinChange(void) {
    static unsigned char hall;
    hall = GetHallPhase();
    IFS1bits.CNBIF = 0;
    
    if(hall && hall != current_phase) {    
        current_phase = hall;
//        T4CONbits.ON = 1;
//        TMR4 = 0;
        
        if(commutate) {
            MotorPhase(current_phase + commutate, motor_speed);
        }    
    }
}

inline void MotorPhase(unsigned char num, unsigned int val) {
    if(num == 0) {
        num = 6;
    } else {
        num = (num - 1) % 6 + 1;
    }
    switch(num) {
        case 0:
            // A - GND
            PDC7 = 0;
            PDC1 = 0;
            // B - V+
            PDC8 = 0;
            PDC2 = 0;
            // C - NC
            PDC9 = 0;
            PDC3 = 0;
            break;
        case 1:
            // A - GND
            PDC7 = PWM_MAX;
            PDC1 = 0;
            // B - V+
            PDC8 = PWM_MAX;
            PDC2 = val;
            // C - NC
            PDC9 = 0;
            PDC3 = 0;
            break;
        case 2:
            // A - GND
            PDC7 = PWM_MAX;
            PDC1 = 0;
            // B - NC
            PDC8 = 0;
            PDC2 = 0;
            // C - V+
            PDC9 = PWM_MAX;
            PDC3 = val;
            break;
        case 3:
            // A - NC
            PDC7 = 0;
            PDC1 = 0;
            // B - GND
            PDC8 = PWM_MAX;
            PDC2 = 0;
            // C - V+
            PDC9 = PWM_MAX;
            PDC3 = val;
            break;
        case 4:
            // A - V+
            PDC7 = PWM_MAX;
            PDC1 = val;
            // B - GND
            PDC8 = PWM_MAX;
            PDC2 = 0;
            // C - NC
            PDC9 = 0;
            PDC3 = 0;
            break;
        case 5:
            // A - V+
            PDC7 = PWM_MAX;
            PDC1 = val;
            // B - NC
            PDC8 = 0;
            PDC2 = 0;
            // C - GND
            PDC9 = PWM_MAX;
            PDC3 = 0;
            break;
        case 6:
            // A - NC
            PDC7 = 0;
            PDC1 = 0;
            // B - V+
            PDC8 = PWM_MAX;
            PDC2 = val;
            // C - GND
            PDC9 = PWM_MAX;
            PDC3 = 0;
            break;           
    }
}

inline unsigned char GetHallPhase() {
    unsigned int hall;
    
    hall = PORTB;
    hall = ((hall >> 7) & 0b100) | ((hall >> 6) & 0b11);
    //hall = PORTBbits.RB8 << 2 | PORTBbits.RB7 << 1 | PORTBbits.RB6;
    
    switch(hall) {
        case 0b101:
            return 4;
            break;
        case 0b001:
            return 5;
            break;
        case 0b011:
            return 6;
            break;
        case 0b010:
            return 1;
            break;
        case 0b110:
            return 2;
            break;
        case 0b100:
            return 3;
            break;
        default:
            return 0;
            break;
    }
}