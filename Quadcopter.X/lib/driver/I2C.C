#include "I2C.h"
#include "settings.h"
#include <stdbool.h>
#include <inttypes.h>
#include <xc.h>

void I2C_init(double freq) {
	double brg;
	I2C5CON = 0;
	I2C5CONbits.DISSLW = 1;

	brg = (1.0 / (2.0 * freq) - 0.000000104) * 100000000.0 - 2.0;
	I2C5BRG = (int)brg;

	I2C5CONbits.ON = 1;
}

bool I2C_WriteRegisters(uint8_t address, uint8_t *data, uint16_t num) {
	bool ret = true;
	
	address <<= 1;

	I2C_Start();
	I2C_Send(address & 0xFE);
	if(!I2C_GetAck()) {
		ret = false;
	}
	while(num--) {
		I2C_Send(*data);
		if(!I2C_GetAck()) {
			ret = false;
		}
		data++;
	}
	I2C_Stop();
	return ret;
}

bool I2C_ReadRegisters(uint8_t address, uint8_t start_adr, uint8_t *data, uint16_t num) {
	bool ret = true;
	
	address <<= 1;

	I2C_Start();
	I2C_Send(address & 0xFE);
	if(!I2C_GetAck()) {
		ret = false;
	}
	I2C_Send(start_adr);
	if(!I2C_GetAck()) {
		ret = false;
	}
	I2C_Restart();
	I2C_Send(address | 0x01);
	if(!I2C_GetAck()) {
		ret = false;
	}
	
	if(num > 1) {
		while(num--) {
			*data = I2C_Read();
			data++;
			if(num > 0) {
				I2C_SendAck();
			}
		}
		
	} else {
		*data = I2C_Read();
	}
	
	I2C_SendNak();
	I2C_Stop();
	
	return ret;
}

bool I2C_CheckAddress(unsigned char addr) {
	bool ret = true;
	
	addr <<= 1;
	
	I2C_Start();
	I2C_Send(addr & 0xFE);
	if(!I2C_GetAck()) {
		ret = false;
	}
	I2C_Stop();
	return ret;
}

void I2C_Send(uint8_t byte) {
	I2C5TRN = byte | 0;
	while(I2C5STATbits.TBF == 1);
	I2C_wait_for_idle();
	// while(I2C1STATbits.ACKSTAT == 1);
}

uint8_t I2C_Read() {
	I2C5CONbits.RCEN = 1;
	while(I2C5CONbits.RCEN);
	while(!I2C5STATbits.RBF);
	uint8_t value = I2C1RCV;

	// if (!ack_nack) {
	// 	I2C_ack();
	// } else {
	// 	I2C_nack();
	// }

	return value;
}

void I2C_wait_for_idle() {
	while(I2C5CON & 0x1F);
	while(I2C5STATbits.TRSTAT);
}

void I2C_Start() {
	I2C_wait_for_idle();
	I2C5CONbits.SEN = 1;
	while (I2C5CONbits.SEN == 1);
}

void I2C_Stop() {
	I2C_wait_for_idle();
	I2C5CONbits.PEN = 1;
}

void I2C_Restart() {
	I2C_wait_for_idle();
	I2C5CONbits.RSEN = 1;
	while (I2C5CONbits.RSEN == 1);
}

bool I2C_GetAck() {
	return !I2C1STATbits.ACKSTAT;
}

void I2C_SendAck() {
	I2C_wait_for_idle();
	I2C5CONbits.ACKDT = 0;
	I2C5CONbits.ACKEN = 1;
	while(I2C5CONbits.ACKEN);
}

void I2C_SendNak() {
	I2C_wait_for_idle();
	I2C5CONbits.ACKDT = 1;
	I2C5CONbits.ACKEN = 1;
	while(I2C5CONbits.ACKEN);
}
