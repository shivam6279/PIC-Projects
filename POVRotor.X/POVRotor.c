#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include <sys/attribs.h>  
#include "pic32.h"
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

void delay_ms(unsigned int x);
void fabulous();
void morph();

#define RPM_TIMER_FREQ 25000

volatile unsigned long int magnet_counter = 0, speed_counter = 0, p_omega = 0, omega = 0, raw_omega = 0;
volatile unsigned char magnet_flag = 1;

volatile double time = 0.0;

struct led cart_image[size][size];

//struct led cart_image2[size][size];

#define RPM_LPF 0.9

void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL4AUTO) speed_timer(void){
    IFS0bits.T3IF = 0;
    
    speed_counter++;
    
    if(PORTDbits.RD4 == 0 && magnet_flag == 1){        
        omega = speed_counter * (1.0-RPM_LPF) + p_omega * RPM_LPF;
        p_omega = omega;
        
        speed_counter = 0;
        magnet_flag = 0;
    }
    else if(PORTDbits.RD4 == 1){
        magnet_flag = 1;
    }
    
//    time += 0.00002;
}

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL3AUTO) LED_timer(void){
    static double angle;
    
    IFS0bits.T4IF = 0;
    if(!LED_TX_INTERRUPT) {        
        angle = 360.0 * ((double)speed_counter)/((double)omega);
//        if(angle > 360.0) angle -= 360.0;
//        if(angle > 360.0) angle -= 360.0;
        polar_image(buffer, cart_image, angle);        
//        scaleBrightness(buffer, 0.3);
        writeLEDs_ISR(buffer); 
    }  
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
    unsigned char gif_frame = 0, max_frames;
    
    PICInit();    
    TRISDbits.TRISD4 = 1;
    
    timer2_init(1000); 
    timer3_init(RPM_TIMER_FREQ);
    timer4_init(5000);
    
    timer5_init(1000000);
    
    RPM_TIMER_ON = 1;
    
    delay_ms(200);
    SPI_init();
    delay_ms(200);
    
    for(j = 0; j < LED_LENGTH; j++)
        buffer[j] = color_red;
    for(j = 0; j < 16; j++){
        writeLEDs(buffer);
        delay_ms(5);
    }
    for(j = 0; j < LED_LENGTH; j++)
        buffer[j] = color_green;
    for(j = 0; j < 16; j++){
        writeLEDs(buffer);
        delay_ms(5);
    }
    for(j = 0; j < LED_LENGTH; j++)
        buffer[j] = color_blue;
    for(j = 0; j < 16; j++){
        writeLEDs(buffer);
        delay_ms(5);
    }
    for(j = 0; j < LED_LENGTH; j++)
        buffer[j] = color_black;
    for(j = 0; j < 16; j++){
        writeLEDs(buffer);
        delay_ms(5);
    }
    
//    led_test_loop();
    
    for(j = 0; j < LED_LENGTH; j++){
        buffer[j] = color_black;
    }
    
    rpm = 0.0;
    do{
        if(omega != 0.0) {
            rpm = (double)RPM_TIMER_FREQ / (double)omega * 60.0;
        }
        writeLEDs(buffer);
        delay_ms(50);
    }while(rpm < 800);
    
    for(i = 0; i < size; i++){
        for(j = 0; j < size; j++){
            cart_image[i][j] =  color_black;
        }
    }
    
//    for(i = 0; i < size; i++){
//        for(j = 0; j < size; j++){
//            cart_image[i][j].red =   ppm[i*size*3 + j*3];
//            cart_image[i][j].green = ppm[i*size*3 + j*3 + 1];
//            cart_image[i][j].blue =  ppm[i*size*3 + j*3 + 2];
//        }
//    }
        
    StartDelayCounter();
    
    LED_TIMER_ON = 1;    
    uS_TIMER_ON = 1;
    
//    struct led temp_color;
//    while(1) {
//        if(ms_counter2() > 100) {
//            if(temp_us < 60) {
//                temp_color = color_blue;
//            } else if(temp_us < 70) {
//                temp_color = color_red;
//            } else if(temp_us < 80) {
//                temp_color = color_green;
//            } else {
//                temp_color = color_white;
//            }
//            for(i = 0; i < size; i++){
//                for(j = 0; j < size; j++){
//                    cart_image[i][j] = temp_color;
//                }
//            }
//        }
//    }
    
    max_frames = gif_init();    
    gif_get_frame(cart_image, 0);
    
    while(1) {
        if(ms_counter2() > 10) {
            set_ms_counter2(0);
            gif_get_frame(cart_image, gif_frame++);
            if(gif_frame >= max_frames) {
                gif_frame = 0;
            }
        }
    }
}