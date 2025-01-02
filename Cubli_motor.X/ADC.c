#include "ADC.h"
#include <sys/attribs.h>
#include <sys/kmem.h>
#include <xc.h>

unsigned int adc_data[5] = {0};

void ADCInit() {
	// ISNS_V       - AN1   - ADC1
	// ISNS_U       - AN9   - ADC4
	// VSNS_12V     - AN8   - ADC3
	// VSNS_VBAT    - AN7   - ADC7
	// ISNS_VIN     - AN27  - ADC7
	
	ANSELAbits.ANSA0 = 1;
	ANSELAbits.ANSA1 = 1;
	ANSELBbits.ANSB0 = 1;
	ANSELBbits.ANSB1 = 1;
	ANSELBbits.ANSB9 = 1;
	ANSELCbits.ANSC0 = 1;
	ANSELCbits.ANSC1 = 1;

	ADC0CFG = DEVADC0;
	ADC1CFG = DEVADC1;
	ADC2CFG = DEVADC2;
	ADC3CFG = DEVADC3;
	ADC4CFG = DEVADC4;
	ADC7CFG = DEVADC7;

	ADCCON1 = 0;
	ADCCON2 = 0;
	ADCCON3 = 0;

	ADCCON1bits.STRGSRC = 0b00001;

	ADCCON2bits.ADCDIV = 0b0000001;

	ADCTRGMODE = 0;
	ADCTRGMODEbits.SH3ALT = 0b10;   // AN8
	ADCTRGMODEbits.SH4ALT = 0b10;   // AN9

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

	// Software edge trigger
	ADCTRG1bits.TRGSRC1 = 0b00001;
	ADCTRG1bits.TRGSRC3 = 0b00001;
	ADCTRG2bits.TRGSRC4 = 0b00001;
	ADCTRG2bits.TRGSRC7 = 0b00001;
	ADCTRG7bits.TRGSRC27 = 0b00001;

	ADCCON1bits.ON = 1;

	while(!ADCCON2bits.BGVRRDY);
	while(ADCCON2bits.REFFLT);

	ADCANCONbits.ANEN1 = 1;
	ADCANCONbits.ANEN3 = 1;
	ADCANCONbits.ANEN4 = 1;
	ADCANCONbits.ANEN7 = 1;

	while(!ADCANCONbits.WKRDY1);
	while(!ADCANCONbits.WKRDY3);
	while(!ADCANCONbits.WKRDY4);
	while(!ADCANCONbits.WKRDY7);

	ADCCON3bits.DIGEN1 = 1;
	ADCCON3bits.DIGEN3 = 1;
	ADCCON3bits.DIGEN4 = 1;
	ADCCON3bits.DIGEN7 = 1;
}

void adc_readAll() {
	ADCCON3bits.GSWTRG = 1;

	while (ADCDSTAT1bits.ARDY1 == 0);
	while (ADCDSTAT1bits.ARDY3 == 0);
	while (ADCDSTAT1bits.ARDY4 == 0);
	while (ADCDSTAT1bits.ARDY7 == 0);
	while (ADCDSTAT1bits.ARDY27 == 0);

	adc_data[0] = ADCDATA1;
	adc_data[1] = ADCDATA3;
	adc_data[2] = ADCDATA4;
	adc_data[3] = ADCDATA7;
	adc_data[4] = ADCDATA27;
}
