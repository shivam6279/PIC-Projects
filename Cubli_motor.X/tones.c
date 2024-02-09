#include "tones.h"
#include <xc.h>
#include "pic32.h"
#include "BLDC.h"
#include <sys/attribs.h>
#include "USART.h"

const float tone_power = 0.05;//0.015
unsigned char tone_phase = 1;

void __ISR_AT_VECTOR(_TIMER_5_VECTOR, IPL3AUTO) tone(void){
    IFS0bits.T5IF = 0;
    MotorPhase(tone_phase++, tone_power);
    if(tone_phase > 2) {
        tone_phase = 1;
    }
}

void MetroidSaveTheme(unsigned char id) {
    if(id == 1) {
        PlayTone(NOTE_D4);
        delay_ms(750);
        PlayTone(NOTE_F4);
        delay_ms(750);
        PlayTone(NOTE_D4);
        delay_ms(750);
        PlayTone(NOTE_C4);
        delay_ms(750);
        PlayTone(NOTE_A3); 
        delay_ms(2000);
        StopTone();
    } else if(id == 2) {
        PlayTone(NOTE_D5);
        delay_ms(750);
        PlayTone(NOTE_F5);
        delay_ms(750);
        PlayTone(NOTE_D5);
        delay_ms(750);
        PlayTone(NOTE_C5);
        delay_ms(750);
        PlayTone(NOTE_A4); 
        delay_ms(2000);
        StopTone();
    } else if(id == 3) {
        PlayTone(NOTE_D6);
        delay_ms(750);
        PlayTone(NOTE_F6);
        delay_ms(750);
        PlayTone(NOTE_D6);
        delay_ms(750);
        PlayTone(NOTE_C6);
        delay_ms(750);
        PlayTone(NOTE_A5); 
        delay_ms(2000);
        StopTone();    
    }
    play_tone = 0;
}

void PlayTone(float freq) {
    float f = 30000000.0 / freq; 
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
    T5CONbits.TCKPS = pre & 0b111;
    PR5 = t;
    TMR5 = 0;
    T5CONbits.ON = 1;
}

void StopTone() {
    T5CONbits.ON = 0;
    MotorOff();
}