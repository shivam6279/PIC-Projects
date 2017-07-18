 #define _XTAL_FREQ 48000000
#include <xc.h>
#include <pic18f4550.h>
#include <math.h>
#include "SPI.h"

#pragma config PLLDIV = 12
#pragma config CPUDIV = OSC1_PLL2
#pragma config USBDIV = 1
#pragma config FOSC = HS
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRT = OFF
#pragma config BOR = OFF
#pragma config BORV = 2
#pragma config VREGEN = OFF
#pragma config WDT = OFF
#pragma config WDTPS = 32768
#pragma config CCP2MX = ON
#pragma config PBADEN = OFF
#pragma config LPT1OSC = OFF
#pragma config MCLRE = ON
#pragma config STVREN = OFF
#pragma config LVP = OFF
#pragma config ICPRT = OFF
#pragma config XINST = OFF
#pragma config CP0 = OFF
#pragma config CP1 = OFF
#pragma config CP2 = OFF
#pragma config CP3 = OFF
#pragma config CPB = OFF
#pragma config CPD = OFF
#pragma config WRT0 = OFF
#pragma config WRT1 = OFF
#pragma config WRT2 = OFF
#pragma config WRT3 = OFF
#pragma config WRTC = OFF
#pragma config WRTB = OFF
#pragma config WRTD = OFF
#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF
#pragma config EBTRB = OFF

unsigned char receive;

volatile unsigned long int speed_counter = 0, speed_offset = 0;
unsigned char flag = 1;

void pwm_init(){
    CCP1CON = 12;
    CCP2CON = 60;
    PR2 = 74;
    T2CON = 7;
    CCPR1L = 0;
    CCPR2L = 0;
}

void delay_ms(unsigned int x){
    unsigned int i;
    for(i = 0; i < x; i++) __delay_ms(1);
}

void init(){
    TRISB = 4;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
}

void timer_init(unsigned char a){
    T0CS = 0;           //a = 7: 1:256
    T0SE = 0;           //a = 6: 1:128
    PSA = 0;            //a = 5: 1:64
    T0PS2 = (a / 4) % 2;//a = 4: 1:32
    T0PS1 = (a / 2) % 2;//a = 3: 1:16
    T0PS0 = a % 2;      //a = 2: 1:8
    IPEN = 0;           //a = 1: 1:4
    GIE = 1;            //a = 0: 1:2
    TMR0 = 0;
    TMR0IE = 1;
    TMR0ON = 1;
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

void interrupt ISR(){
    /*if(RCIF){
        receive = RCREG;
    }*/
    if(TMR0IF){
        TMR0IF = 0;
        speed_counter++;
        if(PORTBbits.RB2 == 0 && flag == 1){
            speed_offset = speed_counter;
            speed_counter = 0;
            flag = 0;
        }
        else if(PORTBbits.RB2 == 1){
            flag = 1;
        }
    }
}

void main(){    
    int i;
    double angle;
    init();
    timer_init(2);
    SPI_init();
    start_frame();
    for(i = 0; i < 23; i++){
        LED_frame(0, 0, 0);
    }
    end_frame();
    while(1){
        angle = 360.0 * ((double)speed_counter)/((double)speed_offset);
        start_frame();
        
        //W
        if(angle > 0.0 && angle < 3.0){
            for(i = 0; i < 5; i++){
                LED_frame(255, 0, 0);
            }
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 3.0 && angle < 6.0){
            LED_frame(0, 0, 0);
            LED_frame(255, 0, 0);
            for(i = 0; i < 21; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 6.0 && angle < 9.0){
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 0, 0);
            for(i = 0; i < 20; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 9.0 && angle < 12.0){
            LED_frame(0, 0, 0);
            LED_frame(255, 0, 0);
            for(i = 0; i < 21; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 12.0 && angle < 15.0){
            for(i = 0; i < 5; i++){
                LED_frame(255, 0, 0);
            }
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        
        if(angle > 180.0 && angle < 183.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            for(i = 0; i < 5; i++){
                LED_frame(255, 0, 0);
            }
        }
        else if(angle > 183.0 && angle < 186.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 0, 0);
            LED_frame(0, 0, 0);
        }
        else if(angle > 186.0 && angle < 189.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
        }
        else if(angle > 189.0 && angle < 192.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 0, 0);
            LED_frame(0, 0, 0);
        }
        else if(angle > 192.0 && angle < 195.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            for(i = 0; i < 5; i++){
                LED_frame(255, 0, 0);
            }
        }
        
        //O
        else if(angle > 18.0 && angle < 21.0){
            for(i = 0; i < 5; i++){
                LED_frame(0, 255, 0);
            }
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 21.0 && angle < 30.0){
            LED_frame(0, 255, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 2550, 0);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 30.0 && angle < 33.0){
            for(i = 0; i < 5; i++){
                LED_frame(0, 255, 0);
            }
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        
        else if(angle > 198.0 && angle < 201.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            for(i = 0; i < 5; i++){
                LED_frame(0, 255, 0);
            }
        }
        else if(angle > 201.0 && angle < 210.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 255, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 255, 0);
        }
        else if(angle > 210.0 && angle < 213.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            for(i = 0; i < 5; i++){
                LED_frame(0, 255, 0);
            }
        }
        
        //R
        else if(angle > 36.0 && angle < 39.0){
            for(i = 0; i < 5; i++){
                LED_frame(0, 0, 255);
            }
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 39.0 && angle < 45.0){
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 255);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 45.0 && angle < 48.0){
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 255);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 48.0 && angle < 51.0){
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 255);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }  
        
        else if(angle > 216.0 && angle < 219.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            for(i = 0; i < 5; i++){
                LED_frame(0, 0, 255);
            }
        }
        else if(angle > 219.0 && angle < 225.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 0);
        }
        else if(angle > 225.0 && angle < 228.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 0);
        }
        else if(angle > 228.0 && angle < 231.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 255);
        }
        
        //K
        else if(angle > 54.0 && angle < 57.0){
            for(i = 0; i < 5; i++){
                LED_frame(255, 255, 0);
            }
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 57.0 && angle < 60.0){
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }  
        else if(angle > 60.0 && angle < 63.0){
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 0);
            LED_frame(0, 0, 0);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }   
        else if(angle > 63.0 && angle < 66.0){
            LED_frame(255, 255, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 0);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        
        else if(angle > 234.0 && angle < 237.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            for(i = 0; i < 5; i++){
                LED_frame(255, 255, 0);
            }
        }
        else if(angle > 237.0 && angle < 240.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
        }  
        else if(angle > 240.0 && angle < 243.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 0);
            LED_frame(0, 0, 0);
        }   
        else if(angle > 243.0 && angle < 246.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(255, 255, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 0);
        }
        
        //I
        else if(angle > 69.0 && angle < 75.0){
            LED_frame(255, 0, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 0, 255);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 75.0 && angle < 78.0){
            LED_frame(255, 0, 255);
            LED_frame(255, 0, 255);
            LED_frame(255, 0, 255);
            LED_frame(255, 0, 255);
            LED_frame(255, 0, 255);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 78.0 && angle < 84.0){
            LED_frame(255, 0, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 0, 255);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        
        else if(angle > 249.0 && angle < 255.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(255, 0, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 0, 255);
        }
        else if(angle > 255.0 && angle < 258.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(255, 0, 255);
            LED_frame(255, 0, 255);
            LED_frame(255, 0, 255);
            LED_frame(255, 0, 255);
            LED_frame(255, 0, 255);
        }
        else if(angle > 258.0 && angle < 264.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(255, 0, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 0, 255);
        }
        
        //N
        else if(angle > 87.0 && angle < 90.0){
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 90.0 && angle < 93.0){
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 255, 255);
            LED_frame(0, 0, 0);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 93.0 && angle < 96.0){
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 255, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 96.0 && angle < 99.0){
            LED_frame(0, 0, 0);
            LED_frame(0, 255, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 99.0 && angle < 102.0){
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        
        else if(angle > 267.0 && angle < 270.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
        }
        else if(angle > 270.0 && angle < 273.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 0, 0);
            LED_frame(0, 255, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
        }
        else if(angle > 273.0 && angle < 276.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 255, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
        }
        else if(angle > 276.0 && angle < 279.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 255, 255);
            LED_frame(0, 0, 0);
        }
        else if(angle > 279.0 && angle < 282.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
            LED_frame(0, 255, 255);
        }
        
        //G
        else if(angle > 105.0 && angle < 108.0){
            LED_frame(255, 255, 255);
            LED_frame(255, 255, 255);
            LED_frame(255, 255, 255);
            LED_frame(255, 255, 255);
            LED_frame(255, 255, 255);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 108.0 && angle < 111.0){
            LED_frame(255, 255, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 255);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 111.0 && angle < 117.0){
            LED_frame(255, 255, 255);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 255);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 255);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        else if(angle > 117.0 && angle < 120.0){
            LED_frame(255, 255, 255);
            LED_frame(255, 255, 255);
            LED_frame(255, 255, 255);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 255);
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
        }
        
        else if(angle > 285.0 && angle < 288.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(255, 255, 255);
            LED_frame(255, 255, 255);
            LED_frame(255, 255, 255);
            LED_frame(255, 255, 255);
            LED_frame(255, 255, 255);
        }
        else if(angle > 288.0 && angle < 291.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(255, 255, 255);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 255);
        }
        else if(angle > 291.0 && angle < 297.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(255, 255, 255);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 255);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 255);
        }
        else if(angle > 297.0 && angle < 300.0){
            for(i = 0; i < 18; i++){
                LED_frame(0, 0, 0);
            }
            LED_frame(255, 255, 255);
            LED_frame(0, 0, 0);
            LED_frame(255, 255, 255);
            LED_frame(255, 255, 255);
            LED_frame(255, 255, 255);
        }
        
        else{
            for(i = 0; i < 23; i++){
                LED_frame(0, 0, 0);
            }
        }
        end_frame();
    }
}

