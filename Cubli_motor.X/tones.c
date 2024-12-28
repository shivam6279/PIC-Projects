#include "tones.h"
#include <xc.h>
#include "pic32.h"
#include "BLDC.h"
#include <sys/attribs.h>
#include "USART.h"

const float tone_power = 0.05;  //0.015
unsigned char tone_phase = 1;

const float song_metroid_theme[5][2] = {{NOTE_D5, 750}, {NOTE_F5, 750}, {NOTE_D5, 750}, {NOTE_C5, 750}, {NOTE_A4, 2000}};

void __ISR_AT_VECTOR(_TIMER_5_VECTOR, IPL2AUTO) tone(void){
    IFS0bits.T5IF = 0;
    MotorPhase(tone_phase, tone_power);
    tone_phase = (tone_phase % 2) + 1;
}

void MetroidSaveTheme(unsigned char id) {
    unsigned int i;
    unsigned int len = sizeof(song_metroid_theme) / sizeof(song_metroid_theme[0]);
    float pitch_factor = 1;

    if(id == 1) {
        pitch_factor = 0.5;

    } else if(id == 2) {
        pitch_factor = 1;

    } else if(id == 3) {
        pitch_factor = 2;
    }

    for(i = 0; i < len; i++) {
        PlayTone(song_metroid_theme[i][0] * pitch_factor);
        delay_ms(song_metroid_theme[i][1]);
    }
    
    StopTone();  
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