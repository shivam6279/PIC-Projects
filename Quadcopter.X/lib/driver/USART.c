#include "USART.h"
#include "settings.h"
#include "string.h"
#include <xc.h>

//XBee
void USART4_init(unsigned long int baud_rate) {    
    TRISBbits.TRISB14 = 1;
    TRISBbits.TRISB15 = 0;
    U1MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U4RXRbits.U4RXR = 0b0010;
    RPB15Rbits.RPB15R = 0b0010;
    CFGCONbits.IOLOCK = 1;
    
    U4BRG = (100000000.0f / (float)baud_rate / 4.0f) - 1;
    
    U4STAbits.ADM_EN = 0;
    U4STAbits.UTXISEL = 0;
    U4STAbits.UTXINV = 0;
    U4STAbits.URXISEL = 2;
    U4STAbits.OERR = 0;
    
    U4MODEbits.SIDL = 0;
    U4MODEbits.IREN = 0;
    U4MODEbits.UEN = 0;
    U4MODEbits.WAKE = 1;
    U4MODEbits.LPBACK = 0;
    U4MODEbits.ABAUD = 0;
    U4MODEbits.RXINV = 0;
    U4MODEbits.BRGH = 1;
    U4MODEbits.PDSEL = 0;
    U4MODEbits.STSEL = 0;
    
    U4MODEbits.ON = 1; 
    U4STAbits.URXEN = 1;
    U4STAbits.UTXEN = 1;
    
    IFS5bits.U4RXIF = 0;
    IPC42bits.U4RXIP = 6;
    IPC42bits.U4RXIS = 0; 
    IEC5bits.U4RXIE = 1;
    
    IFS5bits.U4TXIF = 0;
    IPC43bits.U4TXIP = 6;
    IPC43bits.U4TXIS = 0; 
    XBEE_TX_INTERRUPT = 0;
}

void USART4_send(unsigned char byte) {
    while(U4STAbits.UTXBF);
    U4TXREG = byte;
}

void USART4_send_str(char str[]) {
    int i;
    for(i = 0; str[i] != '\0'; i++) {
        USART4_send(str[i]);
    }
}

void USART4_write_int(int a) {
    long int tens;

    if(a < 0) {
        a *= (-1);
        USART4_send('-');
    }
    //else USART4_send('+');

    for(tens = 1; tens <= a; tens *= 10);
    if(a != 0) tens /= 10;
    for(; tens > 0; tens /= 10) {
        USART4_send(((a / tens) % 10) + 48);
    }
}

void USART4_write_int_hex(unsigned int a) {
    unsigned int temp, factor;
    signed char i;
    char output[8];

    USART4_send_str("0x");
    
    for(i = 0; a != 0; i++, a/= 16) {
        temp = a % 16;
        if (temp < 10) {
            output[i] = temp + 48; 
        } else {
            output[i] = temp + 55;
        }
    }
    
    if(i == 0) {
        output[i++] = '0';
    }
    if(i % 2 == 1) { 
        output[i++] = '0';
    }
    
    for(i = i - 1; i >= 0; i--) {
        USART4_send(output[i]);
    }
}

void USART4_write_float(double a, unsigned char right) {
    unsigned char i;
    long int tens;

    if(a < 0) {
        a *= (-1);
        USART4_send('-');
    } 
    //else USART4_send('+');
    
    if(a >= 1.0) {
        for(tens = 1; tens <= a; tens *= 10);
        tens /= 10;
        for(; tens > 0; tens /= 10)
            USART4_send(((long int)(a / tens) % 10) + 48);
    } else {
        USART4_send('0');
    }

    USART4_send('.');
    for(i = 0, tens = 10; i < right; i++, tens *= 10)
        USART4_send(((long int)(a * tens) % 10) + 48);
}

void USART6_init(unsigned long int baud_rate) {
    TRISDbits.TRISD0 = 1;
    TRISDbits.TRISD9 = 0;
    U5MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U6RXRbits.U6RXR = 0b0011;   //U6RX at RD0
    RPD9Rbits.RPD9R = 0b0100; //U6TX at RD9
    CFGCONbits.IOLOCK = 1;

    U6BRG = (100000000.0f / (float)baud_rate / 4.0f) - 1;
    
    U6STAbits.ADM_EN = 0;
    U6STAbits.UTXISEL = 3;
    U6STAbits.UTXINV = 0;
    U6STAbits.URXISEL = 0;
    U6STAbits.OERR = 0;
    
    U6MODEbits.SIDL = 0;
    U6MODEbits.IREN = 0;
    U6MODEbits.UEN = 0;
    U6MODEbits.WAKE = 1;
    U6MODEbits.LPBACK = 0;
    U6MODEbits.ABAUD = 0;
    U6MODEbits.RXINV = 0;
    U6MODEbits.BRGH = 1;
    U6MODEbits.PDSEL = 0;
    U6MODEbits.STSEL = 0;
    
    U6MODEbits.ON = 1; 
    U6STAbits.URXEN = 1;
    U6STAbits.UTXEN = 0;
    
    IFS5bits.U6RXIF = 0;
    IEC5bits.U6RXIE = 1;
    IPC47bits.U6RXIP = 6;
    IPC47bits.U6RXIS = 0; 
    
    IEC5bits.U6TXIE = 0;
}

void USART3_init(unsigned long int baud_rate) {
    TRISDbits.TRISD11 = 1;
    U3MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U3RXRbits.U3RXR = 0b0111;   //U3RX at RC13
    RPC14Rbits.RPC14R = 0b0001;  //U3TX at RC14
    CFGCONbits.IOLOCK = 1;
    
    U3BRG = (100000000.0f / (float)baud_rate / 4.0f) - 1;
    
    U3STAbits.ADM_EN = 0;
    U3STAbits.UTXISEL = 3;
    U3STAbits.UTXINV = 0;
    U3STAbits.URXISEL = 0;
    U3STAbits.OERR = 0;
    
    U3MODEbits.SIDL = 0;
    U3MODEbits.IREN = 0;
    U3MODEbits.UEN = 0;
    U3MODEbits.WAKE = 1;
    U3MODEbits.LPBACK = 0;
    U3MODEbits.ABAUD = 0;
    U3MODEbits.RXINV = 0;
    U3MODEbits.BRGH = 1;
    U3MODEbits.PDSEL = 0;
    U3MODEbits.STSEL = 0;
    
    U3MODEbits.ON = 1; 
    U3STAbits.URXEN = 1;
    U3STAbits.UTXEN = 0;
    
    IFS4bits.U3RXIF = 0;
    IEC4bits.U3RXIE = 1;
    IPC39bits.U3RXIP = 6;
    IPC39bits.U3RXIS = 0; 
    
    IEC4bits.U3TXIE = 0;
}