#include "PWM.h"
#include <xc.h>
#include <sys/attribs.h>
#include <stdbool.h>

#define DEAD_TIME 5

/*void __ISR(_PWM_PRI_VECTOR, IPL7SOFT) PWM_Primary_event(void) {
	PTCONbits.SEIEN = 0;
	IFS5bits.PWMPEVTIF = 0;
	LATDINV |= 1 << 6;
	PTCONbits.SEIEN = 1;
}*/

unsigned int PWM_MAX;
void PwmInit(float freq) {
//    PWMKEY = 0xABCD
//    PWMKEY = 0x4321
	
	TRISBbits.TRISB10 = 1;
	TRISBbits.TRISB11 = 1;
	TRISBbits.TRISB12 = 1;
	TRISBbits.TRISB13 = 1;
	TRISBbits.TRISB14 = 1;
	TRISBbits.TRISB15 = 1;

	PTCON = 0;
	PTCONbits.PTEN = 0;

	STCON = 0;
	CHOP = 0;

	PTCONbits.PCLKDIV = 0;
	
	CFGCONbits.PWMAPIN1 = 1;
	CFGCONbits.PWMAPIN2 = 1;
	CFGCONbits.PWMAPIN3 = 1;

	PMD4bits.PWM1MD = 0;
	PMD4bits.PWM2MD = 0;
	PMD4bits.PWM3MD = 0;
	PMD4bits.PWM4MD = 0;
	PMD4bits.PWM5MD = 1;
	PMD4bits.PWM6MD = 1;
	PMD4bits.PWM7MD = 0;
	PMD4bits.PWM8MD = 0;
	PMD4bits.PWM9MD = 0;
	PMD4bits.PWM10MD = 1;
	PMD4bits.PWM11MD = 1;
	PMD4bits.PWM12MD = 1;

	PWMCON1 = 0;
	PWMCON2 = 0;
	PWMCON3 = 0;
	
	PWMCON1bits.ECAM = 1;
	PWMCON2bits.ECAM = 1;
	PWMCON3bits.ECAM = 1;
	
	IOCON1 = 0;
	IOCON1bits.PENH = 1;
	IOCON1bits.FLTMOD = 0b11;
	PDC1 = 0;
	DTR1 = DEAD_TIME;
	ALTDTR1 = DEAD_TIME;

	IOCON2 = 0;
	IOCON2bits.PENH = 1;
	IOCON2bits.FLTMOD = 0b11;
	PDC2 = 0;
	DTR2 = DEAD_TIME;
	ALTDTR2 = DEAD_TIME;

	IOCON3 = 0;
	IOCON3bits.PENH = 1;
	IOCON3bits.FLTMOD = 0b11;
	PDC3 = 0;
	DTR3 = DEAD_TIME;
	ALTDTR3 = DEAD_TIME;
	
	PWMCON7 = 0;
	PWMCON8 = 0;
	PWMCON9 = 0;
	
	PWMCON7bits.ECAM = 1;
	PWMCON8bits.ECAM = 1;
	PWMCON9bits.ECAM = 1; 
	
	IOCON7 = 0;
	IOCON7bits.PENH = 1;
	IOCON7bits.POLH = 1;
	IOCON7bits.FLTMOD = 0b11;
	PDC7 = 0;
	DTR7 = DEAD_TIME;
	ALTDTR7 = DEAD_TIME;
	
	IOCON8 = 0;
	IOCON8bits.PENH = 1;
	IOCON8bits.POLH = 1;
	IOCON8bits.FLTMOD = 0b11;
	PDC8 = 0;
	DTR8 = DEAD_TIME;
	ALTDTR8 = DEAD_TIME;
	
	IOCON9 = 0;
	IOCON9bits.PENH = 1;
	IOCON9bits.POLH = 1;
	IOCON9bits.FLTMOD = 0b11;
	PDC9 = 0;
	DTR9 = DEAD_TIME;
	ALTDTR9 = DEAD_TIME;
		
	PWMCON4bits.ECAM = 0;
	IOCON4 = 0;
	PWMCON4 = 0;
	TRIG4 = 0;
	TRGCON4 = 0;
//    TRGCON4bits.STRGIS = 1;
//    PWMCON4bits.TRGIEN = 1;
//    PWMCON4bits.TRGIF = 0;
	PDC4 = 0;
	PWMCON4bits.PWMHIEN = 1;
	PWMCON4bits.PWMHIF = 0;
	IPC44bits.PWM4IP = 7;
	IPC44bits.PWM4IS = 0;
	IFS5bits.PWM4IF = 0;
	IEC5bits.PWM4IE = 1;
	
	float temp = 120000000.0 / (freq * 2.0); // 1,224.5
	PWM_MAX = temp;
	PTPERbits.PTPER = PWM_MAX;
	
	PHASE4 = temp - 100;
	
	PTCONbits.SEVTPS = 1;
	SEVTCMP = 0;
//	PTCONbits.SEIEN = 1;
//	IPC42bits.PWMPEVTIP = 7;
//	IPC42bits.PWMPEVTSIP = 0;
//	IFS5bits.PWMPEVTIF = 0;
//	IEC5bits.PWMPEVTIE = 1;

	PTCONbits.PTEN = 1;
	while(!PTCONbits.PWMRDY);
}

void WritePwm(int num, int val){
	if (val <= 0) {
		val = 0;
	}
	else if (val >= PWM_MAX) {
		val = PWM_MAX;
	}
	switch(num) {
		case 1:
			PDC1 = val;
			break;
		case 2:
			PDC2 = val;
			break;
		case 3:
			PDC3 = val;
			break;
		case 4:
			PDC4 = val;
			break;
		case 5:
			PDC5 = val;
			break;
		case 6:
			PDC6 = val;
			break;
		case 7:
			PDC7 = val;
			break;
		case 8:
			PDC8 = val;
			break;
		case 9:
			PDC9 = val;
			break;
		case 10:
			PDC10 = val;
			break;
		case 11:
			PDC11 = val;
			break;
		case 12:
			PDC12 = val;
			break;
	}
}
