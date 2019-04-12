#include "USART.h"
#include "settings.h"
#include <xc.h>

void USART1_init(unsigned long int baud_rate) {
    #if board_version == 1 || board_version == 2 || board_version == 3
        TRISBbits.TRISB5 = 1;
        TRISBbits.TRISB3 = 0;
        U1MODEbits.ON = 0;
        CFGCONbits.IOLOCK = 0;
        U1RXRbits.U1RXR = 8;
        RPB3Rbits.RPB3R = 1;
        CFGCONbits.IOLOCK = 1;
    #elif board_version == 4
        TRISDbits.TRISD10 = 1;
        TRISDbits.TRISD11 = 0;
        U1MODEbits.ON = 0;
        CFGCONbits.IOLOCK = 0;
        U1RXRbits.U1RXR = 0b0011;
        RPD11Rbits.RPD11R = 0b0001;
        CFGCONbits.IOLOCK = 1;
    #endif
    
    U1BRG = (100000000.0f / (float)baud_rate / 4.0f) - 1;
    
    U1STAbits.ADM_EN = 0;
    U1STAbits.UTXISEL = 3;
    U1STAbits.UTXINV = 0;
    U1STAbits.URXISEL = 2;
    U1STAbits.OERR = 0;
    
    U1MODEbits.SIDL = 0;
    U1MODEbits.IREN = 0;
    U1MODEbits.UEN = 0;
    U1MODEbits.WAKE = 1;
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
    IPC28bits.U1TXIP = 5;
    IPC28bits.U1TXIS = 0; 
    IEC3bits.U1TXIE = 0;
}

void USART1_send(unsigned char byte) {
    while(U1STAbits.UTXBF);
    U1TXREG = byte;
}

void USART1_send_str(char str[]) {
    int i;
    for(i = 0; str[i] != '\0'; i++) {
        USART1_send(str[i]);
    }
}

void USART1_write_int(int a, unsigned char precision) {
    if(a < 0) {
        a *= (-1);
        USART1_send('-');
    } else {
        USART1_send('+');
    }
    if(precision >= 6) USART1_send(((a / 100000) % 10) + 48);
    if(precision >= 5) USART1_send(((a / 10000) % 10) + 48);
    if(precision >= 4) USART1_send(((a / 1000) % 10) + 48);
    if(precision >= 3) USART1_send(((a / 100) % 10) + 48);
    if(precision >= 2) USART1_send(((a / 10) % 10) + 48);
    if(precision >= 1) USART1_send((a % 10) + 48);
}

void USART1_write_float(double a, unsigned char left, unsigned char right) {
    unsigned char i;
    long int tens = 10;
    if(a < 0) {
        a *= (-1);
        USART1_send('-');
    } else {
        USART1_send('+');
    }    
    if(left >= 7) USART1_send(((int)(a / 1000000) % 10) + 48);
    if(left >= 6) USART1_send(((int)(a / 100000) % 10) + 48);
    if(left >= 5) USART1_send(((int)(a / 10000) % 10) + 48);
    if(left >= 4) USART1_send(((int)(a / 1000) % 10) + 48);
    if(left >= 3) USART1_send(((int)(a / 100) % 10) + 48);
    if(left >= 2) USART1_send(((int)(a / 10) % 10) + 48);
    if(left >= 1) USART1_send(((int)a % 10) + 48);
    USART1_send('.');
    for(i = 0; i < right; i++) {
        USART1_send(((long int)(a * tens) % 10) + 48);
        tens *= 10;
    }
}

void USART5_init(unsigned long int baud_rate) {
    TRISDbits.TRISD10 = 1;
    U5MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U5RXRbits.U5RXR = 3;    //U5RX at RD10
    //U5TX select
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