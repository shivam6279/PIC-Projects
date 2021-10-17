#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include <sys/attribs.h>  
#include "bitbang_i2c.h"
#include "USART.h"
#include "SPI.h"
#include "LED.h"
#include "debug.h"
#include "draw.h"
#include "animation.h"

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

void init();
void delay_ms(unsigned int x);
void timer2_init();
void timer3_init(float frequency);
void fabulous();
void morph();

unsigned long int delay_counter = 0;

struct led buffer[LED_LENGTH];
struct led p_buffer[LED_LENGTH];

#define speed_history_size 10

unsigned long int speed_history[speed_history_size];
unsigned int speed_history_i = 0;
volatile unsigned long int magnet_counter = 0, speed_counter = 0, p_omega = 0, omega = 0, raw_omega = 0;
volatile unsigned char magnet_flag = 1;

volatile double time = 0.0;

void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4AUTO) delay_timer(void){
    IFS0bits.T2IF = 0;
    delay_counter++;
}

//void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL4AUTO) speed_timer(void){
//    int i;
//    IFS0bits.T3IF = 0;
//    magnet_counter++;
//    speed_counter++;
//    if(PORTDbits.RD4 == 0 && magnet_flag == 1){
//        speed_history[speed_history_i] = magnet_counter;
//        raw_omega = magnet_counter;
//        for(i = 0, omega = 0; i < speed_history_size; i++){
//            omega += speed_history[i];
//        }
//        omega /= speed_history_size;
//        speed_history_i = (speed_history_i + 1) % speed_history_size;
//        
//        magnet_counter = 0;
//        magnet_flag = 0;
//    }
//    else if(PORTDbits.RD4 == 1){
//        magnet_flag = 1;
//    }
//    if(speed_counter >= omega){
//        speed_counter = 0;
//    }
//    
//    time += 0.00002;
//}

void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL4AUTO) speed_timer(void){
    int i;
    IFS0bits.T3IF = 0;
    
    speed_counter++;
    
    if(PORTDbits.RD4 == 0 && magnet_flag == 1){        
        omega = speed_counter * 0.5 + p_omega * 0.5;
        p_omega = omega;
        
        speed_counter = 0;
        magnet_flag = 0;
    }
    else if(PORTDbits.RD4 == 1){
        magnet_flag = 1;
    }
    
    time += 0.00002;
}

long int mag(long int a){
    if(a < 0) 
        return -a;
    else 
        return a;
}

void main(){
    int i, j, k;
    double angle, angle_offset = 0.0;
    float rpm;
    
    init();    
    TRISDbits.TRISD4 = 1;
    
    for(i = 0; i < speed_history_size; i++){
        speed_history[i] = 0;
    }
    timer2_init(); 
    timer3_init(50000);
    
    delay_ms(200);
    SPI_init();
    SPI1BRG = 2;//5
    delay_ms(200);
    
    for(j = 0; j < LED_LENGTH; j++){
        buffer[j] = color_black;
    }
    rpm = 0.0;
    do{
        if(omega != 0.0) {
            rpm = 50000.0 / (double)omega * 60.0;
        }
        writeLEDs(buffer);
        delay_ms(50);
    }while(rpm < 250);
    
//    struct led array[3] = {color_red, color_green, color_blue};
//    
//    while(1) {
//        for(j = 0; j < LED_LENGTH; j++){
//            buffer[j] = color_black;
//        }
//        
//        rpm = 50000.0 / (double)omega * 60.0;
//        angle = 360.0 * ((double)speed_counter)/((double)omega);
//
//        pie(buffer, array, 3, angle);        
//        
//        writeLEDs(buffer);
//    }
    
    struct led color[6] = {color_red, color_blue, color_green, color_cyan, color_magenta, color_yellow};
    
    struct led cart_image[144][144];
    for(i = 0; i < 144; i++){
        for(j = 0; j < 144; j++){
            if(ppm[i * 144 * 3 + j * 3] == 0xaa){
                cart_image[i][j].red = 0xFF;
                cart_image[i][j].green = 0;
                cart_image[i][j].blue = 0;
            } else {
                cart_image[i][j].red = ppm[i * 144 * 3 + j * 3];
                cart_image[i][j].green = ppm[i * 144 * 3 + j * 3 + 1];
                cart_image[i][j].blue = ppm[i * 144 * 3 + j * 3 + 2];
            }
        }
    }
    
    while(1){        
        for(i = 0; i < LED_LENGTH; i++){
            p_buffer[i] = buffer[i];
        }
        for(i = 0; i < LED_LENGTH; i++){
            buffer[i] = color_black;
        }
        
        angle = 360.0 * ((double)speed_counter)/((double)omega);
        
        if(time > 360.0) time = 0.0;
        
        polar_image(buffer, cart_image, angle);
        
        //polar_image(buffer, cart_image, angle - time * 300);
        //polar_neg_d(buffer, cosn, d_cosn, color[4], (angle + time * 25));
//        for(i = 0; i < LED_LENGTH; i++){
//            if(buffer[i].red != p_buffer[i].red || buffer[i].green != p_buffer[i].green || buffer[i].blue != p_buffer[i].blue) {
//                //writeLEDs(buffer);
//                writeLEDs_hue(buffer, 0);//100
//                break;
//            }
//        }
        writeLEDs_hue(buffer, 100);
    }
}

void init(){
    //IO pins
    TRISB = 0;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    TRISF = 0;
    TRISG = 0;
    ANSELB = 0;
    ANSELE = 0;
    ANSELG = 0;
    
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