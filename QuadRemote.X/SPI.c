#include "SPI.h"
#include <xc.h>

void SPI_init() {
    unsigned char data;
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD10 = 0;
    CFGCONbits.IOLOCK = 0;
    RPD10Rbits.RPD10R = 5;
    CFGCONbits.IOLOCK = 1;
    
    SPI1CONbits.ON = 0;
    SPI1CONbits.FRMEN = 0;
    SPI1CONbits.SIDL = 0;
    SPI1CONbits.MCLKSEL = 0;
    SPI1CONbits.DISSDO = 0;
    SPI1CONbits.MODE32 = 0;
    SPI1CONbits.MODE16 = 0;
    SPI1CONbits.SMP = 0;
    SPI1CONbits.CKE = 1;
    SPI1CONbits.SSEN = 0;
    SPI1CONbits.CKP = 0;
    SPI1CONbits.MSTEN = 1;    
    SPI1CONbits.DISSDI = 1;
    SPI1CONbits.STXISEL = 1;

    SPI1CON2bits.AUDEN = 0;

    SPI1BRG = 0;
    
    data = SPI1BUF;
    SPI1CONbits.ON = 1;
}

void SPI_write(unsigned char data) {
    unsigned char a;
    SPI1BUF = data;
    while(!SPI1STATbits.SPITBE);
    data = SPI1BUF;
}
