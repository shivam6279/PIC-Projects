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
    tens /= 10;
    for(; tens > 0; tens /= 10)
        USART4_send(((a / tens) % 10) + 48);
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

void USART5_init(unsigned long int baud_rate) {
    TRISCbits.TRISC14 = 1;
    TRISCbits.TRISC13 = 0;
    U5MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U5RXRbits.U5RXR = 0b0111;   //U5RX at RC14
    RPC13Rbits.RPC13R = 0b0011; //U5TX at RC13
    CFGCONbits.IOLOCK = 1;

    U5BRG = (100000000.0f / (float)baud_rate / 4.0f) - 1;
    
    U5STAbits.ADM_EN = 0;
    U5STAbits.UTXISEL = 3;
    U5STAbits.UTXINV = 0;
    U5STAbits.URXISEL = 0;
    U5STAbits.OERR = 0;
    
    U5MODEbits.SIDL = 0;
    U5MODEbits.IREN = 0;
    U5MODEbits.UEN = 0;
    U5MODEbits.WAKE = 1;
    U5MODEbits.LPBACK = 0;
    U5MODEbits.ABAUD = 0;
    U5MODEbits.RXINV = 0;
    U5MODEbits.BRGH = 1;
    U5MODEbits.PDSEL = 0;
    U5MODEbits.STSEL = 0;
    
    U5MODEbits.ON = 1; 
    U5STAbits.URXEN = 1;
    U5STAbits.UTXEN = 0;
    
    IFS5bits.U5RXIF = 0;
    IEC5bits.U5RXIE = 1;
    IPC45bits.U5RXIP = 6;
    IPC45bits.U5RXIS = 0; 
    
    IEC5bits.U5TXIE = 0;
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