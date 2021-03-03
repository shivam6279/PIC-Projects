#include "SPI.h"
#include <xc.h>
#include <stdbool.h>
#include "USART.h"
#include "pic32.h"

void SPI_init() {
    unsigned char data;
    TRISBbits.TRISB5 = 1; // SDI
    TRISBbits.TRISB6 = 0; // SCK
    TRISBbits.TRISB7 = 0; // SDO
    CFGCONbits.IOLOCK = 0;
    RPB7Rbits.RPB7R = 0b00100; // SDO2 @ RB7
    SDI2Rbits.SDI2R = 0b0001; //SDI2 @ RB5
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
    SPI2CONbits.STXISEL = 1;    
    SPI2CONbits.ENHBUF = 1;

    SPI2CON2bits.AUDEN = 0;

    SPI2BRG = 0;
    
    data = SPI2BUF;
    SPI2CONbits.ON = 1;
}

unsigned char SPI_write(unsigned char data) {
    SPI2CONbits.MODE32 = 0;
    SPI2CONbits.MODE16 = 0;
    SPI2BUF = data;
    while(!SPI2STATbits.SPIRBE);
    return SPI2BUF;
}

unsigned short SPI_write16(unsigned short data) {
    SPI2CONbits.MODE32 = 0;
    SPI2CONbits.MODE16 = 1;
    SPI2BUF = data;
    while(!SPI2STATbits.SPIRBE);
    return SPI2BUF;
}

#define kNOERROR 0
#define kPRIMARYREADERROR 1
#define kEXTENDEDREADTIMEOUTERROR 2
#define kPRIMARYWRITEERROR 3
#define kEXTENDEDWRITETIMEOUTERROR 4
#define kCRCERROR 5

const unsigned int WRITE = 0x40;
const unsigned int READ = 0x00;
const unsigned int COMMAND_MASK = 0xC0;
const unsigned int ADDRESS_MASK = 0x3F;

void A1339_init() {
    A1339_write(0x3C, 0x2700);
	A1339_write(0x3C, 0x8100);
	A1339_write(0x3C, 0x1F00);
	A1339_write(0x3C, 0x7700);
    
    delay_ms(50);
    
    A1339_write32(0x59, 0b0010000001000000100100);
}

void A1339_test() {
    unsigned short angle;
    unsigned short temperature;
    unsigned short fieldStrength;
    
    while(1) {
        if (A1339_read(0x20, &angle) == kNOERROR) {
            if (CalculateParity(angle)) {
                USART3_send_str("Angle = ");
                USART3_write_int((float)(angle & 0x0FFF) * 360.0 / 4096.0);
                USART3_send_str(" Degrees\n");
            } else {
                USART3_send_str("Parity error on Angle read\n");
            }
        } else {
            USART3_send_str("Unable to read Angle\n");
        }

        if (A1339_read(0x28, &temperature) == kNOERROR) {
            USART3_send_str("Temperature = ");
            USART3_write_float(((float)(temperature & 0x0FFF) / 8.0) + 25.0, 2);
            USART3_send_str(" C\n");
        } else {
            USART3_send_str("Unable to read Temperature\n");
        }

        if (A1339_read(0x2A, &fieldStrength) == kNOERROR) {
            USART3_send_str("Field Strength = ");
            USART3_write_int(fieldStrength & 0x0FFF);
            USART3_send_str(" Gauss\n");
        } else {
            USART3_send_str("Unable to read Field Strength\n");
        }
        delay_ms(500);
    }
}

unsigned short A1339_write(unsigned short addr, unsigned short val) {
    unsigned short command = (((addr & ADDRESS_MASK) | WRITE) << 8)  | ((val >> 8) & 0x0FF);
    unsigned char crcCommand = CalculateCRC(command);
    
    SPI_write16(command);
    SPI_write(crcCommand);
    
    return kNOERROR;
}

unsigned int A1339_write32(unsigned short addr, unsigned int val) {
    unsigned short results;
    unsigned short writeFlags;

	// Write into the extended address register
    results = A1339_write(0x02, addr & 0xFFFF);
    
    if (results != kNOERROR) {
        return results;
    }

	// Write the MSW (Most significant word) into the high order write data register
    results = A1339_write(0x04, (val >> 16) & 0xFFFF);
        
    if (results != kNOERROR) {
        return results;
    }

	// Write the LSW (Least significant word) into the low order write data register
    results = A1339_write(0x06, val & 0xFFFF);
        
    if (results != kNOERROR) {
        return results;
    }

	// Start the write process
    results = A1339_write(0x08, 0x8000);
        
    if (results != kNOERROR) {
        return results;
    }

    StartDelaymsCounter();

	// Wait for the write to complete
    do {
        results = A1339_write(0x08, writeFlags);
    
        if (results != kNOERROR) {
            return results;
        }

        if (ms_counter() > 100) {
            return kEXTENDEDWRITETIMEOUTERROR;
        }
    } while ((writeFlags & 0x0001) != 0x0001);

    return results;
}

unsigned short A1339_read(unsigned short addr, unsigned short *val) {
    unsigned int command = ((addr & ADDRESS_MASK) | READ) << 8;
    unsigned char crcCommand = CalculateCRC(command);
    
    SPI_write16(command);
    SPI_write(crcCommand);
    
    *val = SPI_write16(command);
    unsigned char crcValue = SPI_write(crcCommand);
    
    if (CalculateCRC(*val) != crcValue) {
        return kCRCERROR;
    }
    return kNOERROR;
}

bool CalculateParity(unsigned short input) {
    unsigned short count = 0;
    int index;
    
    // Count up the number of 1s in the input
    for (index = 0; index < 16; ++index)
    {
        if ((input & 1) == 1)
        {
            ++count;
        }

        input >>= 1;
    }
    
    // return true if there is an odd number of 1s
    return (count & 1) != 0;
}

/*
 * CalculateCRC
 *
 * Take the 16 bit input and generate a 4bit CRC
 * Polynomial = x^4 + x^1 + 1
 * LFSR preset to all 1's
 */
unsigned char CalculateCRC(unsigned short input) {
    bool CRC0 = true;
    bool CRC1 = true;
    bool CRC2 = true;
    bool CRC3 = true;
    int  i;
    bool DoInvert;
    uint16_t mask = 0x8000;
   
    for (i = 0; i < 16; ++i)
    {
        DoInvert = ((input & mask) != 0) ^ CRC3;         // XOR required?

        CRC3 = CRC2;
        CRC2 = CRC1;
        CRC1 = CRC0 ^ DoInvert;
        CRC0 = DoInvert;
        mask >>= 1;
    }

    return (CRC3 ? 8U : 0U) + (CRC2 ? 4U : 0U) + (CRC1 ? 2U : 0U) + (CRC0 ? 1U : 0U);
}
