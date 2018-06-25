#ifndef _SPI_H_
#define _SPI_H_

#include <xc.h>

void SPI_init(){
    unsigned char data;
    TRISGbits.TRISG6 = 0;
    TRISDbits.TRISD10 = 0;
    CFGCONbits.IOLOCK = 0;
    RPD10Rbits.RPD10R = 0b0110;
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

#endif


