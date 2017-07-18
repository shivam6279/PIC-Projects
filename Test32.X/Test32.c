#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include <sys/attribs.h>  
#include "bitbang_i2c.h"
#include "PWMDriver.h"
#include "USART.h"
#include "SPI.h"
#include "ColorLCD.h"

// Device Config Bits in  DEVCFG1:		
#pragma config FNOSC = SPLL	
#pragma config FSOSCEN = OFF	
//#pragma config FWDTEN = OFF  
#pragma config POSCMOD = OFF	
#pragma config OSCIOFNC = ON	

// Device Config Bits in  DEVCFG2:		
#pragma config FPLLICLK = PLL_FRC	
#pragma config FPLLIDIV = DIV_1	
#pragma config FPLLMULT = MUL_50	
#pragma config FPLLODIV = DIV_2	
#pragma config FPLLRNG = RANGE_5_10_MHZ	
#pragma config FWDTEN = OFF           
#pragma config FDMTEN = OFF  

#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config TRCEN = ON               // Trace Enable (Trace features in the CPU are enabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = ALLOW_PG2       // Debug Mode CPU Access Permission (Allow CPU access to Permission Group 2 permission regions)
#pragma config EJTAGBEN = NORMAL  

#pragma config PGL1WAY = OFF
#pragma config PMDL1WAY = OFF
#pragma config IOL1WAY = OFF

unsigned long int delay_counter = 0, loop_counter = 0;

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

void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4SRS) delay_timer(void){
    IFS0bits.T2IF = 0;
    delay_counter++;
}

void init(){
    //IO pins
    TRISB = 0;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    TRISF = 0;
    ANSELB = 0;
    
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
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
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

void main(){
    unsigned char i, j;
    unsigned int r, g, b;
    
    //srand(TMR2);
    init();
    timer2_init();
    SPI_init();
    
    
    while(1){
        DC = 1;
        RST = 1;
        CS = 1;
        delay_ms(250);
        DC = 0;
        RST = 0;
        CS = 0;
        delay_ms(250);
    }
    ColorLCD_init();
    delay_ms(120); 		
    ColorLCD_writecommand(0x29);    //Display on 
    ColorLCD_fillRect(0, 0, 320, 250, 0xFFFF);
    while(1){
        for(i = 0; i < 120 ;i += 1){
            r = rand() & 0b11111;
            g = rand() & 0b111111;
            b = rand() & 0b11111;
            ColorLCD_fillRect(i, i, (320 - 2 * i), (240 - 2 * i), (r << 11 | g << 5 | b));
        }
        delay_ms(1000);
    }
}