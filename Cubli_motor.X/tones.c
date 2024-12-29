#include "tones.h"
#include <xc.h>
#include <math.h>
#include "pic32.h"
#include "BLDC.h"
#include "string_utils.h"
#include <sys/attribs.h>
#include "USART.h"

const float tone_power = 0.03;  //0.015
unsigned char tone_phase = 0;

const float song_metroid_theme[5][2] = {{NOTE_D5, 750}, {NOTE_F5, 750}, {NOTE_D5, 750}, {NOTE_C5, 750}, {NOTE_A4, 2000}};

void __ISR_AT_VECTOR(_TIMER_5_VECTOR, IPL2AUTO) tone(void){
    IFS0bits.T5IF = 0;
    MotorPhase(tone_phase, tone_power);
    tone_phase = (tone_phase + 1) % 2;
}

void MetroidSaveTheme(unsigned char id) {
    unsigned int i;
    unsigned int len = sizeof(song_metroid_theme) / sizeof(song_metroid_theme[0]);
    float pitch_factor = 1;

    if(id == 1) {
        pitch_factor = 1.0;

    } else if(id == 2) {
        pitch_factor = 2.0;

    } else if(id == 3) {
        pitch_factor = 4.0;
    }

    for(i = 0; i < len; i++) {
        PlayTone(song_metroid_theme[i][0] * pitch_factor);
        delay_ms(song_metroid_theme[i][1]);
    }
    
    StopTone();  
    play_tone = 0;
}

void PlayTone(float freq) {
    float f = 30000000.0 / (freq / 2); 
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
    
    FOC_TIMER_ON = 0;
    T5CONbits.ON = 1;
}

void StopTone() {
    T5CONbits.ON = 0;
    MotorOff();
}

void PlayNote(const char *in_str) {
    char str[5];
    unsigned char num;
    unsigned char i;
    float freq;
    const float two_12 = 1.05946309436;

    str_cpy(in_str, str);
    str_toUpper(str);    

    i = str_len(str);
    
    if(!char_isDigit(str[i - 1])) {
        return;
    }

    num = str[i - 1] - '0';

    if(str[0] == 'A') {
        freq = NOTE_A0;
    } else if(str[0] == 'B') {
        freq = NOTE_B0;
    } else if(str[0] == 'C') {
        freq = NOTE_C0;
    } else if(str[0] == 'D') {
        freq = NOTE_D0;
    } else if(str[0] == 'E') {
        freq = NOTE_E0;
    } else if(str[0] == 'F') {
        freq = NOTE_F0;
    } else if(str[0] == 'G') {
        freq = NOTE_G0;
    }

    freq *= pow(2, num);

    if(i >= 3) {
        if(str[1] == 'S') {
            freq *= two_12;
        }  else if(str[1] == 'F') {
            freq /= two_12;
        }
    }

    PlayTone(freq);
}