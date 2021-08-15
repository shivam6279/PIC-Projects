#include <xc.h>
#include "XBee.h"
#include "pic32.h"
#include "MPU6050.h"
#include "PID.h"
#include "USART.h"
#include <string.h>
#include <sys/attribs.h>

volatile rx XBee;

volatile unsigned char mode = 'R';

volatile int safety_counter = 0;

static volatile int tx_buffer_index = 0;
static volatile bool XBee_signal_temp = 0;

volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
static unsigned int rx_buffer_index = 0;
volatile unsigned char rx_rdy = 0;

static volatile char tx_buffer[XBEE_TX_BUFFER_LEN];

void __ISR_AT_VECTOR(_UART1_RX_VECTOR, IPL6SRS) XBee_rx(void) {
    IFS3bits.U1RXIF = 0; 

    static unsigned char r;

    do {
        r = U1RXREG & 0xFF;
        if(r == '\r') {
            rx_buffer[rx_buffer_index] = '\0';
            rx_buffer_index = 0;
            
            rx_rdy = 0;
            if((rx_buffer[0] >= '0' && rx_buffer[0] <= '9') || rx_buffer[0] == '-' || rx_buffer[0] == '+') { 
                rx_rdy = 1;
            } else {
                mode = rx_buffer[0];
            }
        } else {
            rx_buffer[rx_buffer_index++] = r;
        }
    }while(U1STAbits.URXDA);
    
    IFS3bits.U1RXIF = 0; 
}

void __ISR_AT_VECTOR(_UART1_TX_VECTOR, IPL6SRS) XBee_tx(void) {
    static int i;

    IFS3bits.U1TXIF = 0;

    if(tx_buffer_index && !U1STAbits.UTXBF) {
        U1TXREG = tx_buffer[0];
        if(tx_buffer_index == 1) {
            tx_buffer_index = 0;
            UART1_TX_INTERRUPT = 0;
        } else {
            for(i = 0; i < tx_buffer_index; i++)
                tx_buffer[i] = tx_buffer[i + 1];
            tx_buffer_index--;
        }
    }
}

void XBeeReset() {
    XBee.x1 = 0;
    XBee.y1 = 0;
    XBee.x2 = 0;
    XBee.y2 = 0;
    XBee.ls = 0;
    XBee.rs = 0;
    XBee.signal = 0;
    XBee.data_ready = 0;
    XBee_signal_temp = 0;
}

rx ReadXBee() {
    return XBee;
}

void XBeeFillBuffer() {
    int i;
    while(!U1STAbits.UTXBF && tx_buffer_index > 0) {
        U1TXREG = tx_buffer[0];
        for(i = 0; i < tx_buffer_index; i++)
            tx_buffer[i] = tx_buffer[i + 1];
        tx_buffer_index--;
    }
    if(tx_buffer_index) {
        IFS3bits.U1TXIF = 0;
        UART1_TX_INTERRUPT = 1;
    }
}

void XBeeClearBuffer() {
    UART1_TX_INTERRUPT = 0;
    tx_buffer[0] = '\0';
    tx_buffer_index = 0;
}

bool TxBufferEmpty() {
    if(tx_buffer_index)
        return 0;
    return 1;
}

void XBeeWriteChar(char a) {
    UART1_TX_INTERRUPT = 0;

    tx_buffer[tx_buffer_index++] = a;
    tx_buffer[tx_buffer_index] = '\0';
    
    XBeeFillBuffer();
}

void XBeeWriteStr(const char str[]) {
    unsigned int i;

    UART1_TX_INTERRUPT = 0;
    
    for(i = 0; str[i] != '\0'; i++, tx_buffer_index++) 
        tx_buffer[tx_buffer_index] = str[i];

    tx_buffer[tx_buffer_index] = '\0';
    
    XBeeFillBuffer();
}

void XBeeWriteInt(int a) {
    long int tens;

    UART1_TX_INTERRUPT = 0;
    
    if(a < 0) { 
        a *= -1; 
        tx_buffer[tx_buffer_index++] = '-'; 
    }
    
    if(a > 1) {
        for(tens = 1; tens < a; tens *= 10);
        tens /= 10;
    } else {
        tens = 1;
    }

    for(; tens > 0; tens /= 10)
        tx_buffer[tx_buffer_index++] = ((long int)(a / tens) % 10) + 48;

    tx_buffer[tx_buffer_index] = '\0';

    XBeeFillBuffer();
}

unsigned char XBeeWriteFloat(float a, unsigned char precision) {
    unsigned char i, len = 0;;
    long int tens;

    UART1_TX_INTERRUPT = 0;
    
    if(a < 0) { 
        a *= -1; 
        tx_buffer[tx_buffer_index++] = '-'; 
        len++;
    }
    
    if(a > 1.0) {
        for(tens = 1; tens < a; tens *= 10);
        tens /= 10;

        for(; tens > 0; tens /= 10, len++)
            tx_buffer[tx_buffer_index++] = ((long int)(a / tens) % 10) + 48;
    } else {
        tx_buffer[tx_buffer_index++] = '0';
        len++;
    }

    tx_buffer[tx_buffer_index++] = '.';
    len++;
    
    for(i = 0, tens = 10; i < precision; i++, tens *= 10, len++)
        tx_buffer[tx_buffer_index++] = ((long int)(a * tens) % 10) + 48;

    tx_buffer[tx_buffer_index] = '\0';

    XBeeFillBuffer();
    
    return len;
}

unsigned char FloatStrLen(float a, unsigned char precision) {
    unsigned char i, len = 0;;
    long int tens;
    
    if(a < 0) { 
        a *= -1;
        len++;
    }
    
    if(a > 1.0) {
        for(tens = 1; tens < a; tens *= 10);
        tens /= 10;

        for(; tens > 0; tens /= 10, len++);
    } else {
        len++;
    }
    len++;
    
    for(i = 0, tens = 10; i < precision; i++, tens *= 10, len++);
    
    return len;
}
