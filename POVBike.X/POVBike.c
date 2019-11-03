#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include <sys/attribs.h>  
#include <math.h>
#include "bitbang_i2c.h"
#include "USART.h"
#include "SPI.h"
#include "LED.h"
#include "debug.h"
#include "draw.h"
#include "animation.h"
#include "10DOF.h"
#include "USART.h"

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
void timer4_init(float frequency);
void fabulous();
void morph();

#define CLOCK_FREQ 256000000.0

unsigned long int delay_counter = 0;

struct led buffer[2][LED_LENGTH + RADIUS_OFFSET];
struct led p_buffer[2][LED_LENGTH + RADIUS_OFFSET];

volatile unsigned long int magnet_counter = 0, omega = 0;
unsigned char magnet_flag = 1;

volatile unsigned long int loop_counter = 0;

volatile double time = 0.0;

#define ANGLE_CORRECTION 35.0

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL7SRS) loop_timer(void){
    IFS0bits.T4IF = 0;
    loop_counter++;
}

void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4SRS) delay_timer(void){
    IFS0bits.T2IF = 0;
    delay_counter++;
}

void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL4SRS) speed_timer(void){
    IFS0bits.T3IF = 0;
    magnet_counter++;
    if(PORTGbits.RG7 == 0 && magnet_flag == 1){
        omega = magnet_counter;
        
        magnet_counter = 0;
        magnet_flag = 0;
    }
    else if(PORTGbits.RG7 == 1){
        magnet_flag = 1;
    }
    
    time += 0.00003;
}

long int mag(long int a){
    if(a < 0) 
        return -a;
    else 
        return a;
}

void main(){
    int i, j, k;
    double angle;
    
    struct led color[6] = {color_red, color_blue, color_green, color_cyan, color_magenta, color_yellow};
    
    init();
    
    ANSELGbits.ANSG7 = 0;
    TRISGbits.TRISG7 = 1;

    timer2_init(); 
    timer3_init(50000);
    timer4_init(1000000.0);
    
    delay_ms(200);
    USART3_init(115200);
    SPI_init();
    SPI4_init();
    SPI2BRG = 3;
    SPI4BRG = 3;
    
    /*
    USART3_send_str("Start\n");
    
    MPU6050Init();
    
    double a = 0.0, dt;
    
    loop_counter = 0;
    T4CONbits.ON = 1;
    while(1) {
        GetAcc();
        GetGyro();
        
        dt = (float)loop_counter / 1000000.0f;
        loop_counter = 0;
        
        a = 1*(a + gyro.z * dt);// + 0.2 * 57.29 * (atan2(-acc.y, -acc.x) + 3.1416);
        limit_angle(&a);
        
        USART3_write_float(a, 2);
        USART3_send_str("\n");
        
        delay_ms(50);
    }*/
    
    if(PORTGbits.RG7) {
        for(i = 0; i < LED_LENGTH; i++){
            buffer[0][i] = color_magenta;
            buffer[1][i] = color_magenta;
        }
    } else {
        for(i = 0; i < LED_LENGTH; i++){
            buffer[0][i] = color_yellow;
            buffer[1][i] = color_yellow;
        }
    }
    
    writeLEDs(buffer);
    
    delay_ms(1000);
    
    while(omega < 800.0){
        writeLEDs(buffer);
        delay_ms(50);
    }
    
    /*
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
    */
    while(1){        
        for(i = 0; i < (LED_LENGTH + RADIUS_OFFSET); i++){
            p_buffer[0][i] = buffer[0][i];
            p_buffer[1][i] = buffer[1][i];
        }
        for(i = 0; i < (LED_LENGTH + RADIUS_OFFSET); i++){
            buffer[0][i] = color_black;
            buffer[1][i] = color_black;
        }
        
        angle = 360.0 * ((double)magnet_counter)/((double)omega);// + ANGLE_CORRECTION;
        limit_angle(&angle);
        
        if(time > 360.0) time = 0.0;
        
        if(angle < 15) {
            for(i = 0; i < LED_LENGTH; i++){
                buffer[0][i] = color_magenta;
                buffer[1][i] = color_magenta;
            }
        } else {
            for(i = 0; i < LED_LENGTH; i++){
                buffer[0][i] = color_black;
                buffer[1][i] = color_black;
            }
        }
        
        writeLEDs(buffer);
        
        //polar_image(buffer, cart_image, angle);
//        
//        pie(buffer, (struct led[15]){color_cyan, color_black, color_black, color_black, color_black, color_black, color_black, color_black, color_black, color_black, color_black, color_black, color_black, color_black, color_black}, 15, angle);
//        
//        for(i = 0; i < (LED_LENGTH + RADIUS_OFFSET); i++){
//            if(buffer[0][i].red != p_buffer[0][i].red || buffer[0][i].green != p_buffer[0][i].green ||buffer[0][i].blue != p_buffer[0][i].blue ||
//            buffer[1][i].red != p_buffer[1][i].red || buffer[1][i].green != p_buffer[1][i].green ||buffer[1][i].blue != p_buffer[1][i].blue) {
//                writeLEDs(buffer);
//                //writeLEDs_hue(buffer, 100);
//                break;
//            }
//        }
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
    float t = CLOCK_FREQ / 2.0 / frequency; unsigned char pre = 0;
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

void timer4_init(float frequency){
    float f = CLOCK_FREQ / 2.0 / frequency; 
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
    T4CONbits.ON = 0;
    T4CONbits.T32 = 0;
    T4CONbits.TCKPS = pre & 0b111;
    PR4 = t;
    TMR4 = 0;
    
    IPC4bits.T4IP = 7;
    IFS0bits.T4IF = 0;
    IEC0bits.T4IE = 1;
}