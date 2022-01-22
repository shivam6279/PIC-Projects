#include "SPI.h"
#include <xc.h>
#include <sys/attribs.h>  

void SPI_init(){
    unsigned char data;
    TRISDbits.TRISD10 = 0;
    TRISDbits.TRISD3 = 0;
    CFGCONbits.IOLOCK = 0;
    RPD3Rbits.RPD3R = 0b1000;
    CFGCONbits.IOLOCK = 1;
    
    SPI4CONbits.ON = 0;
    SPI4CONbits.FRMEN = 0;
    SPI4CONbits.SIDL = 0;
    SPI4CONbits.MCLKSEL = 0;
    SPI4CONbits.DISSDO = 0;
    SPI4CONbits.MODE32 = 0;
    SPI4CONbits.MODE16 = 0;
    SPI4CONbits.SMP = 0;
    SPI4CONbits.CKE = 1;
    SPI4CONbits.SSEN = 0;
    SPI4CONbits.CKP = 0;
    SPI4CONbits.MSTEN = 1;    
    SPI4CONbits.DISSDI = 1;
    
    SPI4CONbits.STXISEL = 0b01;    
    SPI4CONbits.ENHBUF = 1;

    SPI4CON2 = 0;

    SPI4BRG = 0;
    
    IFS5bits.SPI4TXIF = 0;
    LED_TX_INTERRUPT = 0;
    IPC41bits.SPI4TXIP = 6;
    IPC41bits.SPI4TXIS = 0;
    
    data = SPI4BUF;
    SPI4CONbits.ON = 1;
}

void SPI_write(unsigned char data){
    SPI4BUF = data;
    while(!SPI4STATbits.SPITBE);
    data = SPI4BUF;
}