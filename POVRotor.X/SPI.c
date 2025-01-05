#include "SPI.h"
#include <xc.h>
#include <sys/attribs.h>  

#define BRG 3

void SPI2_init(){
	unsigned char data;
	TRISDbits.TRISD10 = 0;
	TRISDbits.TRISD3 = 0;
	CFGCONbits.IOLOCK = 0;
	RPE5Rbits.RPE5R = 0b0110;
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
	SPI2CONbits.ENHBUF  = 1;

	SPI2CON2 = 0;

	SPI2BRG = BRG;
	
	IFS4bits.SPI2TXIF = 0;
	LED_C_TX_INTERRUPT = 0;
	IPC36bits.SPI2TXIP = 6;
	IPC36bits.SPI2TXIS = 0;
	
	data = SPI2BUF;
	SPI2CONbits.ON = 1;
}

void SPI3_init(){
	unsigned char data;
	TRISBbits.TRISB14 = 0;
	TRISFbits.TRISF0 = 0;
	CFGCONbits.IOLOCK = 0;
	RPF0Rbits.RPF0R = 0b0111;
	CFGCONbits.IOLOCK = 1;
	
	SPI3CONbits.ON = 0;
	SPI3CONbits.FRMEN = 0;
	SPI3CONbits.SIDL = 0;
	SPI3CONbits.MCLKSEL = 0;
	SPI3CONbits.DISSDO = 0;
	SPI3CONbits.MODE32 = 0;
	SPI3CONbits.MODE16 = 0;
	SPI3CONbits.SMP = 0;
	SPI3CONbits.CKE = 1;
	SPI3CONbits.SSEN = 0;
	SPI3CONbits.CKP = 0;
	SPI3CONbits.MSTEN = 1;    
	SPI3CONbits.DISSDI = 1;
	
	SPI3CONbits.STXISEL = 0b01;    
	SPI3CONbits.ENHBUF  = 1;

	SPI3CON2 = 0;

	SPI3BRG = BRG;
	
	IFS4bits.SPI3TXIF = 0;
	LED_B_TX_INTERRUPT = 0;
	IPC39bits.SPI3TXIP = 6;
	IPC39bits.SPI3TXIS = 0;
	
	data = SPI3BUF;
	SPI3CONbits.ON = 1;
}

void SPI4_init(){
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
	SPI4CONbits.ENHBUF  = 1;

	SPI4CON2 = 0;

	SPI4BRG = BRG;
	
	IFS5bits.SPI4TXIF = 0;
	LED_A_TX_INTERRUPT = 0;
	IPC41bits.SPI4TXIP = 6;
	IPC41bits.SPI4TXIS = 0;
	
	data = SPI4BUF;
	SPI4CONbits.ON = 1;
}

void SPI2_write(unsigned char data){
	SPI2BUF = data;
	while(!SPI2STATbits.SPITBE);
	data = SPI2BUF;
}

void SPI3_write(unsigned char data){
	SPI3BUF = data;
	while(!SPI3STATbits.SPITBE);
	data = SPI3BUF;
}

void SPI4_write(unsigned char data){
	SPI4BUF = data;
	while(!SPI4STATbits.SPITBE);
	data = SPI4BUF;
}

void SPI_all_write(unsigned char data_4, unsigned char data_3, unsigned char data_2) {
	SPI2BUF = data_2;
	SPI3BUF = data_3;
	SPI4BUF = data_4;
	while(!SPI2STATbits.SPITBE);
	while(!SPI3STATbits.SPITBE);
	while(!SPI4STATbits.SPITBE);
	data_2 = SPI2BUF;
	data_3 = SPI3BUF;
	data_4 = SPI4BUF;
}


void SQI_init() {
	SQI1XCON1bits.DDRDATA = 0;
	SQI1XCON1bits.DDRDUMMY = 0;
	SQI1XCON1bits.DDRMODE = 0;
	SQI1XCON1bits.DDRADDR = 0;
	SQI1XCON1bits.DDRCMD = 0;
	SQI1XCON1bits.DUMMYBYTES = 0;
	SQI1XCON1bits.ADDRBYTES = 0;
	SQI1XCON1bits.TYPEDATA = 0b11;
	SQI1XCON1bits.TYPEDUMMY = 0b11;
	SQI1XCON1bits.TYPEMODE = 0b11;
	SQI1XCON1bits.TYPEADDR = 0b11;
	SQI1XCON1bits.TYPECMD = 0b11;
	
	SQI1XCON2bits.DEVSEL = 0;
	SQI1XCON2bits.MODEBYTES = 0b00;
	
	SQI1CFGbits.CSEN = 0b01;
	SQI1CFGbits.DATAEN = 0b10;
	SQI1CFGbits.LSBF = 0;
	SQI1CFGbits.CPOL = 0;
	SQI1CFGbits.CPHA = 0;
	SQI1CFGbits.MODE = 0b001;
	
	SQI1CONbits.SCHECK = 0;
	SQI1CONbits.DDRMODE = 0;
	SQI1CONbits.DASSERT = 0;
	SQI1CONbits.DEVSEL = 0b00;
	SQI1CONbits.LANEMODE = 0b10;
	SQI1CONbits.CMDINIT = 0b00;
	
	SQI1CLKCONbits.CLKDIV = 1;
	SQI1CLKCONbits.EN = 1;
	
	SQI1CFGbits.SQIEN = 1;
}
