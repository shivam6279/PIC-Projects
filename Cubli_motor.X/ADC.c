#include "ADC.h"
#include <sys/attribs.h>
#include <sys/kmem.h>
#include <inttypes.h>
#include <xc.h>
#include "pic32.h"

volatile float isns_u_offset = 1.65, isns_v_offset = 1.65;

unsigned int adc_data[49] = {4};

uint16_t __attribute__ ((coherent, aligned(16))) adc_buffer[6][2][ADC_BUFFER_LEN];
uint8_t __attribute__ ((coherent, aligned(8))) adc_cnt_buffer[6][2];

extern __inline__ unsigned int __attribute__((always_inline)) virt_to_phys(const void* p) { 
	return (int)p<0?((int)p&0x1fffffffL):(unsigned int)((unsigned char*)p+0x40000000L); 
}

void ADCCalib() {
	uint16_t i;
	const float avg_num = 1000;
	float isns_u = 0, isns_v = 0;
	for(i = 0; i < avg_num; i++) {
		isns_u += ((float)adc_buffer[1][0][0] * ADC_CONV_FACTOR) / avg_num;
		isns_v += ((float)adc_buffer[4][0][0] * ADC_CONV_FACTOR) / avg_num;
		delay_ms(1);
	}
	isns_u_offset = isns_u;
	isns_v_offset = isns_v;
}

void ADCInit() {
	// VSNS_U		- AN2	- ADC2
	// VSNS_V		- AN3	- ADC3
	// VSNS_W		- AN0	- ADC0
	// VSNS_X		- AN6	- ADC5
	
	// ISNS_U		- AN9	- ADC4
	// ISNS_V		- AN1	- ADC1
	
	// VSNS_12V		- AN8	- ADC7
	// VSNS_VBAT	- AN7	- ADC7
	// ISNS_VIN		- AN27	- ADC7
	
	ANSELAbits.ANSA0 = 1;
	ANSELAbits.ANSA1 = 1;
	ANSELAbits.ANSA11 = 1;
	ANSELBbits.ANSB0 = 1;
	ANSELBbits.ANSB1 = 1;
	ANSELBbits.ANSB9 = 1;
	ANSELCbits.ANSC0 = 1;
	ANSELCbits.ANSC1 = 1;
	ANSELCbits.ANSC2 = 1;

	ADC0CFG = DEVADC0;
	ADC1CFG = DEVADC1;
	ADC2CFG = DEVADC2;
	ADC3CFG = DEVADC3;
	ADC4CFG = DEVADC4;
	ADC7CFG = DEVADC7;

	ADCCON1 = 0;
	ADCCON2 = 0;
	ADCCON3 = 0;
	
	ADCCON1bits.SELRES = 0b11;
	ADCCON1bits.STRGSRC = 0b00001;

	ADCCON2bits.ADCDIV = 0b0000001;

	ADCTRGMODE = 0;
	ADCTRGMODEbits.SH0ALT = 0b00;   // AN0
	ADCTRGMODEbits.SH1ALT = 0b00;   // AN1
	ADCTRGMODEbits.SH2ALT = 0b00;   // AN2	
	ADCTRGMODEbits.SH3ALT = 0b00;   // AN3
	ADCTRGMODEbits.SH4ALT = 0b10;   // AN9
	ADCTRGMODEbits.SH5ALT = 0b10;   // AN6

	ADCIMCON1 = 0;
	ADCIMCON2 = 0;
	ADCIMCON3 = 0;

	ADCGIRQEN1 = 0;
	ADCGIRQEN2 = 0;

	ADCCSS1 = 0;
	ADCCSS2 = 0;

	ADCCMPEN1 = 0;
	ADCCMPEN2 = 0;
	ADCCMPEN3 = 0;
	ADCCMPEN4 = 0;

	ADCFLTR1 = 0;
	ADCFLTR2 = 0;
	ADCFLTR3 = 0;
	ADCFLTR4 = 0;

	// Primary Special Event trigger
	ADCTRG1bits.TRGSRC0 = 0x8;
	ADCTRG1bits.TRGSRC1 = 0x8;
	ADCTRG1bits.TRGSRC2 = 0x8;
	ADCTRG1bits.TRGSRC3 = 0x8;
	ADCTRG2bits.TRGSRC4 = 0x8;
	ADCTRG2bits.TRGSRC5 = 0x8;
	// Software edge trigger
	ADCTRG2bits.TRGSRC7 = 0b00001;
	ADCTRG3bits.TRGSRC8 = 0b00001;
	ADCTRG7bits.TRGSRC27 = 0b00001;
	
	// DMA
	uint8_t sample_cnt = 0, adc_buffer_len = ADC_BUFFER_LEN;
	while(adc_buffer_len >>= 1) {
		sample_cnt++;
	}
	ADCCON1bits.DMABL = sample_cnt;
	
	ADC0TIMEbits.SELRES = 0b11;
	ADC1TIMEbits.SELRES = 0b11;
	ADC2TIMEbits.SELRES = 0b11;
	ADC3TIMEbits.SELRES = 0b11;
	ADC4TIMEbits.SELRES = 0b11;
	ADC5TIMEbits.SELRES = 0b11;
	
	ADC0TIMEbits.BCHEN = 1;
	ADC1TIMEbits.BCHEN = 1;
	ADC2TIMEbits.BCHEN = 1;
	ADC3TIMEbits.BCHEN = 1;
	ADC4TIMEbits.BCHEN = 1;
	ADC5TIMEbits.BCHEN = 1;
	
	ADCCNTB = KVA_TO_PA(adc_cnt_buffer);
	ADCDMAB = KVA_TO_PA(adc_buffer);
	
	ADCDSTATbits.DMACEN = 1;
	ADCDSTATbits.DMAEN = 1;

	ADCCON1bits.ON = 1;

	while(!ADCCON2bits.BGVRRDY);
	while(ADCCON2bits.REFFLT);
	
	ADCANCONbits.ANEN0 = 1;
	ADCANCONbits.ANEN1 = 1;
	ADCANCONbits.ANEN2 = 1;
	ADCANCONbits.ANEN3 = 1;
	ADCANCONbits.ANEN4 = 1;
	ADCANCONbits.ANEN5 = 1;
	ADCANCONbits.ANEN7 = 1;
	
	while(!ADCANCONbits.WKRDY0);
	while(!ADCANCONbits.WKRDY1);
	while(!ADCANCONbits.WKRDY2);
	while(!ADCANCONbits.WKRDY3);
	while(!ADCANCONbits.WKRDY4);
	while(!ADCANCONbits.WKRDY5);
	while(!ADCANCONbits.WKRDY7);

	ADCCON3bits.DIGEN0 = 1;
	ADCCON3bits.DIGEN1 = 1;
	ADCCON3bits.DIGEN2 = 1;
	ADCCON3bits.DIGEN3 = 1;
	ADCCON3bits.DIGEN4 = 1;
	ADCCON3bits.DIGEN5 = 1;
	ADCCON3bits.DIGEN7 = 1;
}

void adc_readAll() {
	ADCCON3bits.GSWTRG = 1;
	
	while(ADCDSTAT1bits.ARDY0 == 0);
	while(ADCDSTAT1bits.ARDY1 == 0);
	while(ADCDSTAT1bits.ARDY2 == 0);
	while(ADCDSTAT1bits.ARDY3 == 0);
	while(ADCDSTAT1bits.ARDY4 == 0);
	while(ADCDSTAT1bits.ARDY5 == 0);
	while(ADCDSTAT1bits.ARDY7 == 0);
	while(ADCDSTAT1bits.ARDY27 == 0);
	
	adc_data[0] = ADCDATA0;
	adc_data[1] = ADCDATA1;
	adc_data[2] = ADCDATA2;
	adc_data[3] = ADCDATA3;
	adc_data[4] = ADCDATA4;
	adc_data[5] = ADCDATA5;
	adc_data[7] = ADCDATA7;
	adc_data[8] = ADCDATA8;
	adc_data[27] = ADCDATA27;
}
