#include "SPI.h"
#include "pic32.h"
#include <xc.h>
#include <inttypes.h>
#include <sys/attribs.h>
#include <sys/kmem.h>

volatile uint16_t spi_angle = 0;

//void __ISR_AT_VECTOR(_SPI1_TX_VECTOR, IPL5SOFT) SPI_TX(void) {
//	IFS1bits.SPI1TXIF = 0;
//	spi_angle = SPI1BUF & 0xFFF;
//	SPI1BUF = 0x20 << 8;
//}

void __ISR_AT_VECTOR(_SPI1_RX_VECTOR, IPL5SOFT) SPI_RX(void) {
	IFS1bits.SPI1RXIF = 0;
	spi_angle = SPI1BUF & 0xFFF;
//	SPI1BUF = 0x20 << 8;
}

void __ISR(_TIMER_7_VECTOR, IPL5SOFT) SPI_timer(void) {
	IFS2bits.T7IF = 0;
//	T7CONbits.ON = 0;
//	SPI1BUF = 0x20 << 8;
	LATDINV |= 1 << 6;
}

void SPI1_init(float freq) {
	unsigned char data;
	TRISBbits.TRISB7 = 0;	// SCK
	TRISBbits.TRISB6 = 0;	// SS
	TRISCbits.TRISC7 = 1;	// SDI
	TRISBbits.TRISB5 = 0;	// SDO
	CFGCONbits.IOLOCK = 0;
	RPB6Rbits.RPB6R = 0b00011;	// SS
	SDI1Rbits.SDI1R = 0b0101;	// SDI
	RPB5Rbits.RPB5R = 0b00011;	// SDO
	CFGCONbits.IOLOCK = 1;
	
	CNPUBbits.CNPUB6 = 1;
	
	SPI1CON = 0;
//	SPI1CONbits.CKE = 1;
	SPI1CONbits.CKP = 1;
	SPI1CONbits.MSTEN = 1;
	SPI1CONbits.MSSEN = 1;
	SPI1CONbits.MODE16 = 1;
	SPI1CONbits.STXISEL = 0b00;
	SPI1CONbits.SRXISEL = 0b01;
	SPI1CONbits.ENHBUF = 1;

	SPI1CON2 = 0;

	float brg = 60000000.0f / (2.0 * freq) - 1.0;
	SPI1BRG = brg;
	
	IFS1bits.SPI1TXIF = 0;
	IEC1bits.SPI1TXIE = 0;
	IPC9bits.SPI1TXIP = 5;
	IPC9bits.SPI1TXIS = 0;
	
	IFS1bits.SPI1RXIF = 0;
	IEC1bits.SPI1RXIE = 0;
	IPC9bits.SPI1RXIP = 5;
	IPC9bits.SPI1RXIS = 0;
	
	data = SPI1BUF;
	SPI1CONbits.ON = 1;
}

void SPI1_write16(uint16_t data) {
	// SPI1BUF = data;
	// while(!SPI1STATbits.SPITBE);
	// data = SPI1BUF;

	while(!SPI1STATbits.SPITBE);
	SPI1BUF = data;
}
