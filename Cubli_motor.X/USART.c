#include "USART.h"
#include <string.h>
#include <xc.h>
#include <sys/attribs.h>

//----------------------------UART3----------------------------

volatile unsigned char rx_buffer[RX_BUFFER_SIZE];

static unsigned int rx_buffer_index = 0;

volatile unsigned char rx_rdy = 0;

void __ISR_AT_VECTOR(_UART3_RX_VECTOR, IPL6AUTO) UART_DIN(void) {
    static unsigned int r;
    do {
        r = U3RXREG & 0xFF;
        if(r == '\r') {
            rx_buffer[rx_buffer_index] = '\0';
            rx_buffer_index = 0;
            rx_rdy = 1;
        } else {
            rx_buffer[rx_buffer_index++] = r;
        }        
    }while(U3STAbits.URXDA);    
    
    if(U3STAbits.OERR)
        U3STAbits.OERR = 0;
    
    IFS1bits.U3RXIF = 0; 
}

void USART3_init(unsigned long int baud_rate) {
    TRISBbits.TRISB4 = 1;
    TRISAbits.TRISA4 = 0;
    
    U3MODE = 0;
    
    CFGCONbits.IOLOCK = 0;
    U3RXRbits.U3RXR = 0b0010;
    RPA4Rbits.RPA4R = 0b0001;
    CFGCONbits.IOLOCK = 1;
    
    U3BRG = (60000000.0f / (float)baud_rate / 16.0f) - 1;
    
    U3STAbits.ADDR = 0;
    U3STAbits.UTXISEL = 0;
    U3STAbits.UTXINV = 0;
    U3STAbits.URXISEL = 0;
    U3STAbits.URXEN = 1;
    U3STAbits.UTXEN = 1;
    
    U3MODEbits.CLKSEL = 0;    
    U3MODEbits.SIDL = 0;
    U3MODEbits.IREN = 0;
    U3MODEbits.UEN = 0;
    U3MODEbits.LPBACK = 0;
    U3MODEbits.ABAUD = 0;
    U3MODEbits.RXINV = 0;
    U3MODEbits.BRGH = 0;
    U3MODEbits.PDSEL = 0;
    U3MODEbits.STSEL = 0;
//    U3MODEbits.RUNOVF = 1;
    
    IFS1bits.U3RXIF = 0;
    IPC15bits.U3RXIP = 6;
    IPC15bits.U3RXIS = 0; 
    IEC1bits.U3RXIE = 1;
    
    IFS2bits.U3TXIF = 0;
    IPC16bits.U3TXIP = 6;
    IPC16bits.U3TXIS = 0; 
    
    U3MODEbits.ON = 1;    
}

void USART3_send(unsigned char byte) {
    while(U3STAbits.UTXBF);
    U3TXREG = byte;
}

void USART3_send_str(char str[]) {
    int i;
    for(i = 0; str[i] != '\0'; i++) {
        USART3_send(str[i]);
    }
}

void USART3_write_int(long int a) {
    long int tens;

    if(a < 0) {
        a *= (-1);
        USART3_send('-');
    }
    //else USART1_send('+');
    
    for(tens = 1; tens <= a; tens *= 10);
    if(tens >= 10) tens /= 10;
    for(; tens > 0; tens /= 10)
        USART3_send(((a / tens) % 10) + 48);
}

void USART3_write_float(double a, unsigned char right) {
    unsigned char i;
    long int tens;

    if(a < 0) {
        a *= (-1);
        USART3_send('-');
    } 
    //else USART3_send('+');
    
    if(a > 1.0) {
        for(tens = 1; tens <= a; tens *= 10);
        tens /= 10;
        for(; tens > 0; tens /= 10)
            USART3_send(((long int)(a / tens) % 10) + 48);
    } else {
        USART3_send('0');
    }

    USART3_send('.');
    for(i = 0, tens = 10; i < right; i++, tens *= 10)
        USART3_send(((long int)(a * tens) % 10) + 48);
}

//----------------------------UART2----------------------------
