#include "SPI.h"
#include <xc.h>
#include <sys/attribs.h>  

#define LED_BRIGTHNESS 0.3
static unsigned char br_byte = 0b11100000 | (unsigned char)((float)LED_BRIGTHNESS*31);

void SPI_init(){
    unsigned char data;
    TRISGbits.TRISG6 = 0;
    TRISGbits.TRISG8 = 0;
    CFGCONbits.IOLOCK = 0;
    RPG8Rbits.RPG8R = 0b0110;
    CFGCONbits.IOLOCK = 1;
    
    SPI2CONbits.ON = 0;
    SPI2CONbits.FRMEN = 0;
    SPI2CONbits.SIDL = 0;
    SPI2CONbits.MCLKSEL = 0;
    SPI2CONbits.DISSDO = 0;
    SPI2CONbits.MODE32 = 0;
    SPI2CONbits.MODE16 = 0;
    SPI2CONbits.SMP = 0;
    SPI2CONbits.CKE = 1;
    SPI2CONbits.SSEN = 0;
    SPI2CONbits.CKP = 0;
    SPI2CONbits.MSTEN = 1;    
    SPI2CONbits.DISSDI = 1;
    
    SPI2CONbits.STXISEL = 0b01;    
    SPI2CONbits.ENHBUF = 1;

    SPI2CON2 = 0;

    SPI2BRG = 0;
    
    data = SPI2BUF;
    SPI2CONbits.ON = 1;
}

void SPI_write(unsigned char data){
    SPI2BUF = data;
    while(!SPI2STATbits.SPITBE);
    data = SPI2BUF;
}

void start_frame() {
    SPI_write(0);
    SPI_write(0);
    SPI_write(0);
    SPI_write(0);
}

void end_frame() {
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    
    SPI_write(255);
    SPI_write(255);    
    SPI_write(255);
    SPI_write(255);
}

void LED_frame(unsigned char red, unsigned char green, unsigned char blue) {
    SPI_write(br_byte);
    SPI_write(blue);
    SPI_write(green);
    SPI_write(red);
}

void Write_Onboard_LEDs(unsigned char red, unsigned char green, unsigned char blue) {
    start_frame();
    LED_frame(red, green, blue);
    LED_frame(red, green, blue);
    LED_frame(red, green, blue);
    LED_frame(red, green, blue);
    end_frame();
}