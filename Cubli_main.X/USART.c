#include "USART.h"
#include "string.h"
#include <xc.h>

void USART1_init(unsigned long int baud_rate) {
    TRISDbits.TRISD10 = 1;
    TRISDbits.TRISD11 = 0;
    U1MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U1RXRbits.U1RXR = 0b0011;
    RPD11Rbits.RPD11R = 0b0001;
    CFGCONbits.IOLOCK = 1;
    
    U1BRG = (100000000.0f / (float)baud_rate / 4.0f) - 1;
    
    U1STAbits.ADM_EN = 0;
    U1STAbits.UTXISEL = 0;
    U1STAbits.UTXINV = 0;
    U1STAbits.URXISEL = 2;
    U1STAbits.OERR = 0;
    
    U1MODEbits.SIDL = 0;
    U1MODEbits.IREN = 0;
    U1MODEbits.UEN = 0;
    U1MODEbits.WAKE = 0;
    U1MODEbits.LPBACK = 0;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.RXINV = 0;
    U1MODEbits.BRGH = 1;
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;
    
    U1MODEbits.ON = 1; 
    U1STAbits.URXEN = 1;
    U1STAbits.UTXEN = 1;
    
    IFS3bits.U1RXIF = 0;
    IPC28bits.U1RXIP = 6;
    IPC28bits.U1RXIS = 0; 
    IEC3bits.U1RXIE = 1;
    
    IFS3bits.U1TXIF = 0;
    IPC28bits.U1TXIP = 6;
    IPC28bits.U1TXIS = 0; 
    UART1_TX_INTERRUPT = 0;
}

void USART2_init(unsigned long int baud_rate) {
    TRISBbits.TRISB7 = 1;
    TRISBbits.TRISB6 = 0;
    U3MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U2RXRbits.U2RXR = 0b0111;
    RPB6Rbits.RPB6R = 0b0010;
    CFGCONbits.IOLOCK = 1;
    
    U2BRG = (100000000.0f / (float)baud_rate / 16.0f) - 1;
    
    U2STA = 0;
    
    U2MODE = 0;
    
    U2STAbits.URXEN = 1;
    U2STAbits.UTXEN = 1;
    
    IFS4bits.U2RXIF = 0;
    IEC4bits.U2RXIE = 0;
    IPC36bits.U2RXIP = 6;
    IPC36bits.U2RXIS = 0; 
    
    IEC4bits.U2TXIE = 0;
    
    U2MODEbits.ON = 1; 
}

void USART3_init(unsigned long int baud_rate) {
    TRISGbits.TRISG7 = 1;
    TRISGbits.TRISG8 = 0;
    U3MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U3RXRbits.U3RXR = 0b0001;
    RPG8Rbits.RPG8R = 0b0001;
    CFGCONbits.IOLOCK = 1;
    
    U3BRG = (100000000.0f / (float)baud_rate / 16.0f) - 1;
    
    U3STA = 0;
    
    U3MODE = 0;
    
    U3STAbits.URXEN = 1;
    U3STAbits.UTXEN = 1;
    
    IFS4bits.U3RXIF = 0;
    IEC4bits.U3RXIE = 0;
    IPC39bits.U3RXIP = 6;
    IPC39bits.U3RXIS = 0; 
    
    IEC4bits.U3TXIE = 0;
    
    U3MODEbits.ON = 1; 
}

void USART4_init(unsigned long int baud_rate) {
    TRISBbits.TRISB14 = 1;
    TRISBbits.TRISB8 = 0;
    U3MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U4RXRbits.U4RXR = 0b0010;
    RPB8Rbits.RPB8R = 0b0010;
    CFGCONbits.IOLOCK = 1;
    
    U4BRG = (100000000.0f / (float)baud_rate / 16.0f) - 1;
    
    U4STA = 0;
    
    U4MODE = 0;
    
    U4STAbits.URXEN = 1;
    U4STAbits.UTXEN = 1;
    
    IFS5bits.U4RXIF = 0;
    IEC5bits.U4RXIE = 0;
    IPC42bits.U4RXIP = 6;
    IPC42bits.U4RXIS = 0; 
    
    IEC5bits.U4TXIE = 0;
    
    U4MODEbits.ON = 1; 
}

void USART_send(unsigned char port, unsigned char byte) {
    switch(port) {
        case 1:
            while(U1STAbits.UTXBF);
            U1TXREG = byte;
            break;
        case 2:
            while(U2STAbits.UTXBF);
            U2TXREG = byte;
            break;
        case 3:
            while(U3STAbits.UTXBF);
            U3TXREG = byte;
            break;    
        case 4:
            while(U4STAbits.UTXBF);
            U4TXREG = byte;
            break;
        case 5:
            while(U5STAbits.UTXBF);
            U5TXREG = byte;
            break;
    }
}

void USART_send_str(unsigned char port, char str[]) {
    int i;
    for(i = 0; str[i] != '\0'; i++) {
        USART_send(port, str[i]);
    }
}

void USART_write_int(unsigned char port, int a) {
    long int tens;
    
    if(a == 0) {
        USART_send(port, '0');
        return;
    }

    if(a < 0) {
        a *= (-1);
        USART_send(port, '-');
    }
    //else USART_send(port, '+');

    for(tens = 1; tens <= a; tens *= 10);
    tens /= 10;
    for(; tens > 0; tens /= 10)
        USART_send(port, ((a / tens) % 10) + 48);
}

void USART_write_float(unsigned char port, double a, unsigned char right) {
    unsigned char i;
    long int tens;

    if(a < 0) {
        a *= (-1);
        USART_send(port, '-');
    } 
    //else USART_send(port, '+');
    
    if(a > 1.0) {
        for(tens = 1; tens < a; tens *= 10);
        tens /= 10;
        for(; tens > 0; tens /= 10)
            USART_send(port, ((long int)(a / tens) % 10) + 48);
    } else {
        USART_send(port, '0');
    }

    USART_send(port, '.');
    for(i = 0, tens = 10; i < right; i++, tens *= 10)
        USART_send(port, ((long int)(a * tens) % 10) + 48);
}