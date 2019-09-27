#include "SPI.h"

void SPI_init(){
    unsigned char data;
    TRISGbits.TRISG6 = 0;
    TRISGbits.TRISG8 = 0;
    CFGCONbits.IOLOCK = 0;
    RPG8Rbits.RPG8R = 0b0110;
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

void SPI_write(unsigned char data){
    unsigned char a;
    SPI2BUF = data;
    while(!SPI2STATbits.SPITBE);
    data = SPI2BUF;
}