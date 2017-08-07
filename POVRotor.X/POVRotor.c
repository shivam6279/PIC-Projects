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

unsigned long int delay_counter = 0;

const int strip_length = 23;

int morph_counter;

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

void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4SRS) delay_timer(void){
    IFS0bits.T2IF = 0;
    delay_counter++;
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
}

void LED_frame(unsigned char red, unsigned char green, unsigned char blue){
    SPI_write(255);
    SPI_write(blue);
    SPI_write(green);
    SPI_write(red);
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

void main(){
    int i;
    init();
    timer2_init();
    
    delay_ms(200);
    SPI_init();
    morph_counter = 0;
    /*while(1){
        morph();
        delay_ms(10);
    }*/
    start_frame();
    for(i = 0; i < 23; i++){
        LED_frame(255, 0, 255);
    }
    end_frame();
    while(1);
}
