#include "SPI.h"
#include "LED.h"
#include <xc.h>
#include <sys/attribs.h>  

#define BRG 3

extern __inline__ unsigned int __attribute__((always_inline)) virt_to_phys(const void* p) { 
	return (int)p<0?((int)p&0x1fffffffL):(unsigned int)((unsigned char*)p+0x40000000L); 
}

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
	
//	IFS4bits.SPI2TXIF = 0;
//	LED_C_TX_INTERRUPT = 0;
//	IPC36bits.SPI2TXIP = 1; // 6
//	IPC36bits.SPI2TXIS = 0;
	
	data = SPI2BUF;
	SPI2CONbits.ON = 1;

	// ------------------- DMA -------------------
	DCH0CONbits.CHEN = 0;							// Turn off this channel

	DCH0SSA = virt_to_phys(LED_C_tx_buffer);		// Move the data from the [buffer] array
	DCH0DSA = virt_to_phys((const void*)&SPI2BUF);	// Move the data to the x register
	DCH0SSIZ = BUFFER_LENGTH;						// Move num_bytes bytes of data in total
	DCH0CSIZ = 1;									// Move 1 byte at a time
	DCH0DSIZ = 1;									// Destination size is 1 byte

	DCH0ECON=0;										// Clear the DMA configuration settings 
	DCH0ECONbits.CHSIRQ = _SPI2_TX_VECTOR;			// Move data on PMP interrupt
	DCH0ECONbits.CHAIRQ = _SPI2_FAULT_VECTOR;		// Abort on PMP error
	DCH0ECONbits.SIRQEN = 1;						// Enable Start IRQ
//	DCH0ECONbits.AIRQEN = 1;						// Enable Abort IRQ
	
	DCH0CONbits.CHAEN = 0;
	DCH0CONbits.CHPRI = 3;							// The priority of this channel is 3 (highest)
	DCH0CONbits.CHEN = 1;							// Turn this channel on now

	IPC33bits.DMA0IP = 3;							// Set DMA 0 interrupt priority to 3
	IPC33bits.DMA0IS = 1;							// Set DMA 0 interrupt sub-priority to 1
	IFS4bits.SPI2TXIF = 0;							// Clear the SPI TX interrupt flag
	IFS4bits.DMA0IF = 0;							// Clear the DMA channel 0 interrupt flag
	IEC4bits.DMA0IE = 1;							// Enable the DMA 0 interrupt
	DCH0INTbits.CHBCIE = 1;							// Enable the Channel Block Transer Complete (CHBC) Interrupt

//	DCH0ECONbits.CFORCE = 1;						// Force the start of the transfer now

	DMACONSET = 0x8000;
	// ------------------------------------------
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
	
//	IFS4bits.SPI3TXIF = 0;
//	LED_B_TX_INTERRUPT = 0;
//	IPC39bits.SPI3TXIP = 6;
//	IPC39bits.SPI3TXIS = 0;
	
	data = SPI3BUF;
	SPI3CONbits.ON = 1;
	
	// ------------------- DMA -------------------
	DCH1CONbits.CHEN = 0;							// Turn off this channel

	DCH1SSA = virt_to_phys(LED_B_tx_buffer);		// Move the data from the [buffer] array
	DCH1DSA = virt_to_phys((const void*)&SPI3BUF);	// Move the data to the x register
	DCH1SSIZ = BUFFER_LENGTH;						// Move num_bytes bytes of data in total
	DCH1CSIZ = 1;									// Move 1 byte at a time
	DCH1DSIZ = 1;									// Destination size is 1 byte

	DCH1ECON=0;										// Clear the DMA configuration settings 
	DCH1ECONbits.CHSIRQ = _SPI3_TX_VECTOR;			// Move data on PMP interrupt
	DCH1ECONbits.CHAIRQ = _SPI3_FAULT_VECTOR;		// Abort on PMP error
	DCH1ECONbits.SIRQEN = 1;						// Enable Start IRQ
//	DCH1ECONbits.AIRQEN = 1;						// Enable Abort IRQ
	
	DCH1CONbits.CHAEN = 0;
	DCH1CONbits.CHPRI = 3;							// The priority of this channel is 3 (highest)
	DCH1CONbits.CHEN = 1;							// Turn this channel on now

	IPC33bits.DMA1IP = 3;							// Set DMA 0 interrupt priority to 3
	IPC33bits.DMA1IS = 1;							// Set DMA 0 interrupt sub-priority to 1
	IFS4bits.SPI3TXIF = 0;							// Clear the SPI TX interrupt flag
	IFS4bits.DMA1IF = 0;							// Clear the DMA channel 0 interrupt flag
	IEC4bits.DMA1IE = 1;							// Enable the DMA 0 interrupt
	DCH1INTbits.CHBCIE = 1;							// Enable the Channel Block Transer Complete (CHBC) Interrupt

//	DCH0ECONbits.CFORCE = 1;						// Force the start of the transfer now

	DMACONSET = 0x8000;
	// ------------------------------------------
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
	
//	IFS5bits.SPI4TXIF = 0;
//	LED_A_TX_INTERRUPT = 0;
//	IPC41bits.SPI4TXIP = 6;
//	IPC41bits.SPI4TXIS = 0;
	
	data = SPI4BUF;
	SPI4CONbits.ON = 1;
	
	// ------------------- DMA -------------------
	DCH0CONbits.CHEN = 0;							// Turn off this channel

	DCH2SSA = virt_to_phys(LED_A_tx_buffer);		// Move the data from the [buffer] array
	DCH2DSA = virt_to_phys((const void*)&SPI4BUF);	// Move the data to the x register
	DCH2SSIZ = BUFFER_LENGTH;						// Move num_bytes bytes of data in total
	DCH2CSIZ = 1;									// Move 1 byte at a time
	DCH2DSIZ = 1;									// Destination size is 1 byte

	DCH2ECON=0;										// Clear the DMA configuration settings 
	DCH2ECONbits.CHSIRQ = _SPI4_TX_VECTOR;			// Move data on PMP interrupt
	DCH2ECONbits.CHAIRQ = _SPI4_FAULT_VECTOR;		// Abort on PMP error
	DCH2ECONbits.SIRQEN = 1;						// Enable Start IRQ
//	DCH2ECONbits.AIRQEN = 1;						// Enable Abort IRQ
	
	DCH2CONbits.CHAEN = 0;
	DCH2CONbits.CHPRI = 3;							// The priority of this channel is 3 (highest)
	DCH2CONbits.CHEN = 1;							// Turn this channel on now

	IPC34bits.DMA2IP = 3;							// Set DMA 0 interrupt priority to 3
	IPC34bits.DMA2IS = 1;							// Set DMA 0 interrupt sub-priority to 1
	IFS5bits.SPI4TXIF = 0;							// Clear the SPI TX interrupt flag
	IFS4bits.DMA0IF = 0;							// Clear the DMA channel 0 interrupt flag
	IEC4bits.DMA0IE = 1;							// Enable the DMA 0 interrupt
	DCH0INTbits.CHBCIE = 1;							// Enable the Channel Block Transer Complete (CHBC) Interrupt

//	DCH0ECONbits.CFORCE = 1;						// Force the start of the transfer now

	DMACONSET = 0x8000;
	// ------------------------------------------
}

void __ISR_AT_VECTOR(_DMA0_VECTOR, IPL3SRS) DMA0_handler(void) {
    IFS4bits.DMA0IF = 0;	// Clear the DMA channel 0 interrupt flag
//    IEC4bits.DMA0IE = 0;	// Disable the DMA 0 interrupt
	IFS4bits.SPI2TXIF = 0;
//	LATDINV = 1 << 3;
}

void __attribute__((vector(_DMA1_VECTOR), interrupt(IPL3SRS), nomips16)) DMA1_handler(){
    IFS4bits.DMA1IF = 0;	// Clear the DMA channel 0 interrupt flag
//    IEC4bits.DMA0IE = 0;	// Disable the DMA 0 interrupt
	IFS4bits.SPI3TXIF = 0;
//	LATDINV = 1 << 3;
}

void __attribute__((vector(_DMA2_VECTOR), interrupt(IPL3SRS), nomips16)) DMA2_handler(){
    IFS4bits.DMA2IF = 0;	// Clear the DMA channel 0 interrupt flag
//    IEC4bits.DMA2IE = 0;	// Disable the DMA 0 interrupt
	IFS5bits.SPI4TXIF = 0;
//	LATDINV = 1 << 3;
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
