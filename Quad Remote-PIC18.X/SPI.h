#ifndef _SPI_H_
#define _SPI_H_

void SPI_init(){
    SSPEN = 0;
    SMP = 0;
    CKE = 1;
    SSPCON1 = 0b00100000;
    SSPEN = 1;
    SSPIF = 0;
}

void SPI_write(unsigned char data){
    SSPBUF = data;
    while(!SSPIF);
    SSPIF = 0;
}

#endif