#include "USART.h"
#include <string.h>
#include <xc.h>
#include <math.h>
#include <sys/attribs.h>
#include <stdbool.h>
#include "pic32.h"
#include "BLDC.h"

//----------------------------UART3----------------------------

volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
static unsigned int rx_buffer_index = 0;
volatile unsigned char rx_rdy = 0;
volatile unsigned char play_tone = 0;
volatile unsigned char auto_stop = 1;

void __ISR_AT_VECTOR(_UART3_RX_VECTOR, IPL3AUTO) UART_DIN(void) {
    static unsigned int r;
    static bool overflow = false;
    do {
        r = U3RXREG & 0xFF;
       // USART3_send(r);
        if(r == '\r') {
            rx_buffer[rx_buffer_index] = '\0';
            rx_buffer_index = 0;

            if(!overflow) {
                rx_rdy = 1;
            }

            overflow = false;
        } else if(!overflow) {
            rx_buffer[rx_buffer_index++] = r;
            if(rx_buffer_index >= (RX_BUFFER_SIZE - 1)) {
                overflow = true;
                rx_buffer_index = 0;
            }
        }        
    }while(U3STAbits.URXDA);    
    
   // if(U3STAbits.OERR) {
   //     U3STAbits.OERR = 0;
   // }
    
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
    U3MODEbits.RUNOVF = 1;
    
    IFS1bits.U3RXIF = 0;
    IPC15bits.U3RXIP = 3;
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

void USART3_send_str(const char *str) {
    for(; *str != '\0'; str++) {
        USART3_send(*str);
    }
}

void USART3_write_int(long int a) {
    long int tens;

    if(a < 0) {
        a = fabs(a);
        USART3_send('-');
    }
    
    for(tens = 1; tens <= a; tens *= 10);
    if(tens >= 10) tens /= 10;
    for(; tens > 0; tens /= 10)
        USART3_send(((a / tens) % 10) + '0');
}

void USART3_write_float(double a, unsigned char right) {
    unsigned char i;
    long int tens;
    
    if(a < 0.0) {
        a = -a;
        if(a > 1/pow(10, right)) {
            USART3_send('-');
        }
    } 
    
    if(a >= 1.0) {
        for(tens = 1; tens <= a; tens *= 10);
        tens /= 10;
        for(; tens > 0; tens /= 10)
            USART3_send(((long int)(a / tens) % 10) + '0');
    } else {
        USART3_send('0');
    }

    USART3_send('.');
    for(i = 0, tens = 10; i < right; i++, tens *= 10) {
        USART3_send(((long int)(a * tens) % 10) + '0');
    }
}
