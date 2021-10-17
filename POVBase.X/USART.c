#include "USART.h"
#include <string.h>
#include <xc.h>
#include <sys/attribs.h>
#include "pic32.h"
#include "BLDC.h"

//----------------------------UART3----------------------------

volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
static unsigned int rx_buffer_index = 0;
volatile unsigned char rx_rdy = 0;
volatile unsigned char play_tone = 0;

void __ISR_AT_VECTOR(_UART4_RX_VECTOR, IPL7AUTO) UART_DIN(void) {
    static unsigned int r;
    
    do {
        r = U4RXREG & 0xFF;
        if(r == '\r') {
            rx_buffer[rx_buffer_index] = '\0';
            rx_buffer_index = 0;
            
            rx_rdy = 1;
        } else {
            rx_buffer[rx_buffer_index++] = r;
        }        
        
    }while(U4STAbits.URXDA);    
    
    if(U4STAbits.OERR)
        U4STAbits.OERR = 0;
    
    IFS2bits.U4RXIF = 0; 
}

void USART4_init(unsigned long int baud_rate) {
    TRISEbits.TRISE15 = 1;
    TRISAbits.TRISA8 = 0;
    
    U4MODE = 0;
    
    CFGCONbits.IOLOCK = 0;
    U4RXRbits.U4RXR = 0b1000;
    RPA8Rbits.RPA8R = 0b00010;
    CFGCONbits.IOLOCK = 1;
    
    U4BRG = (60000000.0f / (float)baud_rate / 16.0f) - 1;
    
    U4STAbits.ADDR = 0;
    U4STAbits.UTXISEL = 0;
    U4STAbits.UTXINV = 0;
    U4STAbits.URXISEL = 0;
    U4STAbits.URXEN = 1;
    U4STAbits.UTXEN = 1;
    
    U4MODEbits.CLKSEL = 0;    
    U4MODEbits.SIDL = 0;
    U4MODEbits.IREN = 0;
    U4MODEbits.UEN = 0;
    U4MODEbits.LPBACK = 0;
    U4MODEbits.ABAUD = 0;
    U4MODEbits.RXINV = 0;
    U4MODEbits.BRGH = 0;
    U4MODEbits.PDSEL = 0;
    U4MODEbits.STSEL = 0;
//    U4MODEbits.RUNOVF = 1;
    
    IFS2bits.U4RXIF = 0;
    IPC16bits.U4RXIP = 7;
    IPC16bits.U4RXIS = 0; 
    IEC2bits.U4RXIE = 1;
    
    IFS2bits.U4TXIF = 0;
    IPC16bits.U4TXIP = 6;
    IPC16bits.U4TXIS = 0; 
    
    U4MODEbits.ON = 1;    
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

void USART4_write_int(long int a) {
    long int tens;

    if(a < 0) {
        a *= (-1);
        USART4_send('-');
    }
    //else USART1_send('+');
    
    for(tens = 1; tens <= a; tens *= 10);
    if(tens >= 10) tens /= 10;
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
    //else USART3_send('+');
    
    if(a > 1.0) {
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
