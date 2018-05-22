#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include <sys/attribs.h>  
#include "bitbang_i2c.h"
#include "USART.h"
#include "SPI.h"

#pragma config FNOSC = SPLL 
#pragma config FSOSCEN = OFF    
#pragma config POSCMOD = OFF    
#pragma config OSCIOFNC = ON        
#pragma config FPLLICLK = PLL_FRC   
#pragma config FPLLIDIV = DIV_1 
#pragma config FPLLMULT = MUL_50    
#pragma config FPLLODIV = DIV_2 
#pragma config FPLLRNG = RANGE_5_10_MHZ 
#pragma config FWDTEN = OFF           
#pragma config FDMTEN = OFF  
#pragma config DEBUG = OFF           
#pragma config JTAGEN = OFF         
#pragma config ICESEL = ICS_PGx1        
#pragma config TRCEN = ON        
#pragma config BOOTISA = MIPS32        
#pragma config FECCCON = OFF_UNLOCKED  
#pragma config FSLEEP = OFF            
#pragma config DBGPER = ALLOW_PG2      
#pragma config EJTAGBEN = NORMAL  
#pragma config PGL1WAY = OFF
#pragma config PMDL1WAY = OFF
#pragma config IOL1WAY = OFF

#define strip_length 73.0
const float brightness = 0.2;

void init();
void delay_ms(unsigned int x);
void timer2_init();
void timer3_init(float frequency);
void start_frame();
void end_frame();
void LED_frame(unsigned char red, unsigned char green, unsigned char blue);
void fabulous();
void morph();

unsigned long int delay_counter = 0;

struct led{
    unsigned char red, green, blue;
};

struct led buffer[(int)strip_length];
struct led p_buffer[(int)strip_length];
unsigned char buffer_magnet_flag = 0;
int morph_counter = 0;
int fabulous_counter = 0;

#define speed_history_size 4

unsigned long int speed_history[speed_history_size];
unsigned int speed_history_i = 0;
unsigned long int magnet_counter = 0, speed_counter = 0, omega = 0, raw_omega;
unsigned char magnet_flag = 1;

void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4SRS) delay_timer(void){
    IFS0bits.T2IF = 0;
    delay_counter++;
}

void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL4SRS) speed_timer(void){
    int i;
    IFS0bits.T3IF = 0;
    magnet_counter++;
    speed_counter++;
    if(PORTDbits.RD2 == 0 && magnet_flag == 1){
        speed_history[speed_history_i] = magnet_counter;
        raw_omega = magnet_counter;
        for(i = 0, omega = 0; i < speed_history_size; i++){
            omega += speed_history[i];
        }
        omega /= speed_history_size;
        speed_history_i = (speed_history_i + 1) % speed_history_size;
        
        magnet_counter = 0;
        magnet_flag = 0;
    }
    else if(PORTDbits.RD2 == 1){
        magnet_flag = 1;
    }
    if(speed_counter >= omega){
        speed_counter = 0;
    }
}

long int mag(long int a){
    if(a < 0) return -a;
    else return a;
}

void main(){
    int i, j, k;
    int r, g, b;
    unsigned char red, green, blue;
    double angle, t;
    float morph_counter = 0;
    init();
    
    TRISDbits.TRISD2 = 1;
    for(i = 0; i < speed_history_size; i++){
        speed_history[i] = 0;
    }
    timer2_init();
    timer3_init(200000);
    
    delay_ms(200);
    SPI_init();
    SPI2BRG = 5;
    delay_ms(200);
    
    start_frame();
    for(i = 0; i < (int)strip_length; i++){
        LED_frame(255, 255, 255);
    }
    end_frame();
    delay_ms(50);
    
    
    /*long int temp1, temp2;
    while(1){
        temp1 = mag(speed_history[speed_history_i] - speed_history[(speed_history_i - 1) % speed_history_size]);
        temp2 = raw_omega;
        for(i = 0; i < (int)strip_length; i++){
            buffer[i].red = 0;
            buffer[i].green = 0;
            buffer[i].blue = 0;
        }
        for(i = 0; i < 32; i++){
            if(temp1 & 1){
                buffer[i].red = 255;
            } 
            if(temp2 & 1){
                buffer[i].blue = 255;
            } 
            temp1 = temp1 >> 1;
            temp2 = temp2 >> 1;
        }
        
        start_frame();
        for(i = 0; i < (int)strip_length; i++){
            LED_frame(buffer[i].red, buffer[i].green, buffer[i].blue);
        }
        end_frame();
        buffer_magnet_flag = 0;
    }*/
    
    while(1){
        angle = 365.0 * ((double)speed_counter)/((double)omega);
        
        if(morph_counter < 256){
            r = 255;
            g = morph_counter;
            b = 0;
        }
        else if(morph_counter < 512){
            r = 255 - (morph_counter - 256);
            g = 255;
            b = 0;
        }
        else if(morph_counter < 768){
            r = 0;
            g = 255;
            b = morph_counter - 512;
        }
        else if(morph_counter < 1024){
            r = 0;
            g = 255 - (morph_counter - 768);
            b = 255;
        }
        else if(morph_counter < 1280){
            r = morph_counter - 1024;
            g = 0;
            b = 255;
        }
        else{
            r = 255;
            g = 0;
            b = 255 - (morph_counter - 1280);
        }
        
        for(i = 0; i < (int)strip_length; i++){
            p_buffer[i].red = buffer[i].red;
            p_buffer[i].green = buffer[i].green;
            p_buffer[i].blue = buffer[i].blue;
        }
        for(i = 0; i < (int)strip_length; i++){
            buffer[i].red = 0;
            buffer[i].green = 0;
            buffer[i].blue = 0;
        }
        if(angle > 89.0 && angle < 91.0){
            for(i = 0; i < (int)strip_length / 2; i++){
                buffer[i].red = r;
                buffer[i].green = g;
                buffer[i].blue = b;
            }
            morph_counter += 2;
            if(morph_counter >= 1536.0) morph_counter = 0.0;
        }
        else if(angle > 269.0 && angle < 271.0){
            for(i = (int)strip_length / 2 + 1; i < (int)strip_length; i++){
                buffer[i].red = r;
                buffer[i].green = g;
                buffer[i].blue = b;
            }
            morph_counter += 2;
            if(morph_counter >= 1536.0) morph_counter = 0.0;
        }
        for(i = 0; i < (int)strip_length; i++){
            if(buffer[i].red != p_buffer[i].red || buffer[i].green != p_buffer[i].green || buffer[i].blue != p_buffer[i].blue){
                buffer_magnet_flag = 1;
                break;
            }
        }
        
        if(buffer_magnet_flag == 1){
            start_frame();
            for(i = 0; i < (int)strip_length; i++){
                LED_frame(buffer[i].red, buffer[i].green, buffer[i].blue);
            }
            end_frame();
            buffer_magnet_flag = 0;
        }
    }
    //--------------------------------------------------------------------------
    /*
    for(i = 0; i < 9; i++){
        buffer[i].red = 255;
        buffer[i].green = 0;
        buffer[i].blue = 0;
        buffer[(int)strip_length - i].red = 255;
        buffer[(int)strip_length - i].green = 0;
        buffer[(int)strip_length - i].blue = 0;
    }
    for(i = 9; i < 17; i++){
        buffer[i].red = 240;
        buffer[i].green = 240;
        buffer[i].blue = 240;
        buffer[(int)strip_length - i].red = 240;
        buffer[(int)strip_length - i].green = 240;
        buffer[(int)strip_length - i].blue = 240;
    }
    for(i = 17; i < 25; i++){
        buffer[i].red = 255;
        buffer[i].green = 0;
        buffer[i].blue = 0;
        buffer[(int)strip_length - i].red = 255;
        buffer[(int)strip_length - i].green = 0;
        buffer[(int)strip_length - i].blue = 0;
    }    
    for(i = 31; i < 37; i++){
        buffer[i].red = 240;
        buffer[i].green = 240;
        buffer[i].blue = 240;
        buffer[(int)strip_length - i].red = 240;
        buffer[(int)strip_length - i].green = 240;
        buffer[(int)strip_length - i].blue = 240;
    }   
    start_frame();
    for(i = 0; i < (int)strip_length; i++){
        LED_frame(buffer[i].red, buffer[i].green, buffer[i].blue);
    }
    end_frame();
    while(1){
        angle = 360.0 * ((double)magnet_counter)/((double)omega);
        for(i = 25; i < 31; i++){
            buffer[i].red = 0;
            buffer[i].green = 45;
            buffer[i].blue = 92;
            buffer[(int)strip_length - i].red = 0;
            buffer[(int)strip_length - i].green = 45;
            buffer[(int)strip_length - i].blue = 92;
        }
        
        for(j = 0; j < 6; j++){
            if(((int)angle%71 > 11.5*j && (int)angle%71 < (71 - 11.5*j))){
                if(j > 5){
                    buffer[25].red = 240;
                    buffer[25].green = 240;
                    buffer[25].blue = 240;
                    buffer[(int)strip_length - 25].red = 240;
                    buffer[(int)strip_length - 25].green = 240;
                    buffer[(int)strip_length - 25].blue = 240;
                }
                if(j > 4){
                    buffer[26].red = 240;
                    buffer[26].green = 240;
                    buffer[26].blue = 240;
                    buffer[(int)strip_length - 26].red = 240;
                    buffer[(int)strip_length - 26].green = 240;
                    buffer[(int)strip_length - 26].blue = 240;
                }
                if(j > 3){
                    buffer[27].red = 240;
                    buffer[27].green = 240;
                    buffer[27].blue = 240;
                    buffer[(int)strip_length - 27].red = 240;
                    buffer[(int)strip_length - 27].green = 240;
                    buffer[(int)strip_length - 27].blue = 240;
                }
                if(j > 2){
                    buffer[28].red = 240;
                    buffer[28].green = 240;
                    buffer[28].blue = 240;
                    buffer[(int)strip_length - 28].red = 240;
                    buffer[(int)strip_length - 28].green = 240;
                    buffer[(int)strip_length - 28].blue = 240;
                }
                if(j > 1){
                    buffer[29].red = 240;
                    buffer[29].green = 240;
                    buffer[29].blue = 240;
                    buffer[(int)strip_length - 29].red = 240;
                    buffer[(int)strip_length - 29].green = 240;
                    buffer[(int)strip_length - 29].blue = 240; 
                }
                if(j > 0){
                    buffer[30].red = 240;
                    buffer[30].green = 240;
                    buffer[30].blue = 240;
                    buffer[(int)strip_length - 30].red = 240;
                    buffer[(int)strip_length - 30].green = 240;
                    buffer[(int)strip_length - 30].blue = 240; 
                }
            }
        }
        
        start_frame();
        for(i = 0; i < (int)strip_length; i++){
            LED_frame(buffer[i].red, buffer[i].green, buffer[i].blue);
        }
        end_frame();
    }
    
    fabulous_counter = 0;
    //--------------------------------------------------------------------------
    */
    while(1){
        fabulous();
        start_frame();
        for(i = 0; i < (int)strip_length; i+=2){
            LED_frame(buffer[i].red, buffer[i].green, buffer[i].blue);
        }
        LED_frame(buffer[i-2].red, buffer[i-2].green, buffer[i-2].blue);
        for(i = (int)strip_length - 1; i >= 2; i-=2){
            LED_frame(buffer[i].red, buffer[i].green, buffer[i].blue);
        }
        end_frame();
        delay_ms(12);
    }
}

void morph(){
    int r, g, b, i;
    if(morph_counter < 256){
        r = 255;
        g = morph_counter;
        b = 0;
    }
    else if(morph_counter < 512){
        r = 255 - (morph_counter - 256);
        g = 255;
        b = 0;
    }
    else if(morph_counter < 768){
        r = 0;
        g = 255;
        b = morph_counter - 512;
    }
    else if(morph_counter < 1024){
        r = 0;
        g = 255 - (morph_counter - 768);
        b = 255;
    }
    else if(morph_counter < 1280){
        r = morph_counter - 1024;
        g = 0;
        b = 255;
    }
    else{
        r = 255;
        g = 0;
        b = 255 - (morph_counter - 1280);
    }
    start_frame();
    for(i = 0; i < strip_length; i++){
        LED_frame(r, g, b);
    }
    end_frame();
    morph_counter += 1;
    if(morph_counter == 1536){
        morph_counter = 0;
    }
}


void fabulous(){
    int j, k, r, g, b;
    const float ka = 255.0 / strip_length * 6;
    //start_frame();
    for(j = fabulous_counter, k = 0; j < strip_length; j++, k++){
        if(j < 12){
            r = 255;
            g = (int)(float)(ka * j);
            b = 0;
        }
        else if(j < 24){
            r = (int)(float)(ka * (12 - (j - 12)));
            g = 255;
            b = 0;
        }
        else if(j < 36){
            r = 0;
            g = 255;
            b = (int)(float)(ka * (j - 36));
        }
        else if(j < 49){
            r = 0;
            g = (int)(float)(ka * (12 - (j - 36)));
            b = 255;
        }
        else if(j < 61){
            r = (int)(float)(ka * (j - 49));
            g = 0;
            b = 255;
        }
        else{
            r = 255;
            g = 0;
            b = (int)(float)(ka * (12 - (j - 61)));
        }
        //LED_frame(r, g, b);
        buffer[k].red = r;
        buffer[k].green = g;
        buffer[k].blue = b;
    }
    for(j = 0; j < fabulous_counter; j++, k++){
        if(j < 12){
            r = 255;
            g = (int)(float)(ka * j);
            b = 0;
        }
        else if(j < 24){
            r = (int)(float)(ka * (12 - (j - 12)));
            g = 255;
            b = 0;
        }
        else if(j < 36){
            r = 0;
            g = 255;
            b = (int)(float)(ka * (j - 36));
        }
        else if(j < 49){
            r = 0;
            g = (int)(float)(ka * (12 - (j - 36)));
            b = 255;
        }
        else if(j < 61){
            r = (int)(float)(ka * (j - 49));
            g = 0;
            b = 255;
        }
        else{
            r = 255;
            g = 0;
            b = (int)(float)(ka * (12 - (j - 61)));
        }
        //LED_frame(r, g, b);
        buffer[k].red = r;
        buffer[k].green = g;
        buffer[k].blue = b;
    }
    //end_frame();
    fabulous_counter ++;
    if(fabulous_counter == strip_length){
        fabulous_counter = 0;
    }
}

void init(){
    //IO pins
    TRISB = 0xE040;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0xF0;
    TRISF = 0;
    TRISG = 0x0180;
    ANSELB = 0xC000;
    ANSELE = 0xF0;
    ANSELG = 0x0180;
    
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

void delay_ms(unsigned int x){
    delay_counter = 0;
    T2CONbits.TON = 1;
    while(delay_counter < x);
    T2CONbits.TON = 0;
}

void timer2_init(){
    T2CONbits.TON = 0;
    T2CONbits.TCKPS = 5;//1Khz
    PR2 = 3125;
    TMR2 = 0;
    IPC2bits.T2IP = 4;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    T2CONbits.TON = 0;
}

void timer3_init(float frequency){
    float t = 100000000.0 / frequency; unsigned char pre = 0;
    while(t > 65535){ t /= 2.0; pre++; }
    t = (int)t;
    while((int)t % 2 == 0 && pre < 8){ t /= 2.0; pre++; }
    if(pre == 7){ t *= 2.0; pre--; }
    if(pre == 8) pre = 7;
    T3CONbits.ON = 0;
    T3CONbits.TCKPS = pre;
    PR3 = (int)t - 1;
    TMR3 = 0;
    IPC3bits.T3IP = 4;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    T3CONbits.ON = 1;
}

void start_frame(){
    SPI_write(0);
    SPI_write(0);
    SPI_write(0);
    SPI_write(0);
}

void end_frame(){
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
}

void LED_frame(unsigned char red, unsigned char green, unsigned char blue){
    float r = (float)red * brightness;
    float g = (float)green * brightness;
    float b = (float)blue * brightness;
    SPI_write(255);
    SPI_write(b);
    SPI_write(g);
    SPI_write(r);
}