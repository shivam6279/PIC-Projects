#include "SPI.h"

void SPI_init(){
    unsigned char data;
    TRISGbits.TRISG6 = 0;
    TRISEbits.TRISE5 = 0;
    CFGCONbits.IOLOCK = 0;
    RPE5Rbits.RPE5R = 0b0110;
    CFGCONbits.IOLOCK = 1;
    
    SPI2CON = 0;
    SPI2CONbits.CKE = 1;
    SPI2CONbits.MSTEN = 1;    
    SPI2CONbits.DISSDI = 1;
    SPI2CONbits.STXISEL = 1;

    SPI2CON2bits.AUDEN = 0;

    SPI2BRG = 0;
    
    data = SPI2BUF;
    SPI2CONbits.ON = 1;
}

void SPI4_init(){
    unsigned char data;
    TRISDbits.TRISD10 = 0;
    TRISDbits.TRISD11 = 0;
    CFGCONbits.IOLOCK = 0;
    RPD11Rbits.RPD11R = 0b1000;
    CFGCONbits.IOLOCK = 1;
    
    SPI4CON = 0;
    SPI4CONbits.CKE = 1;
    SPI4CONbits.MSTEN = 1;    
    SPI4CONbits.DISSDI = 1;
    SPI4CONbits.STXISEL = 1;

    SPI4CON2bits.AUDEN = 0;

    SPI4BRG = 0;
    
    data = SPI4BUF;
    SPI4CONbits.ON = 1;
}

void SPI_write(unsigned char data){
    SPI2BUF = data;
    while(!SPI2STATbits.SPITBE);
    data = SPI2BUF;
}

void SPI4_write(unsigned char data){
    SPI4BUF = data;
    while(!SPI4STATbits.SPITBE);
    data = SPI4BUF;
}

void SPI_write_double(unsigned char a, unsigned char b) {
    SPI2BUF = a;
    SPI4BUF = b;
    while(!SPI2STATbits.SPITBE || !SPI4STATbits.SPITBE);
    a = SPI2BUF;
    b = SPI4BUF;
}