#include "USART.h"
#include "string.h"
#include <xc.h>
#include <sys/attribs.h>

volatile unsigned char rx_buffer_A[RX_BUFFER_SIZE], rx_buffer_B[RX_BUFFER_SIZE], rx_buffer_C[RX_BUFFER_SIZE];
static unsigned int rx_buffer_index_A = 0, rx_buffer_index_B = 0, rx_buffer_index_C = 0;

volatile float rpm_A = 0.0, rpm_B = 0.0, rpm_C = 0.0;

float parse_rx(volatile char* buffer) {
    int i;
    float ret;
    unsigned long int tens;
    unsigned char flag = 0;
    
    unsigned char temp_buffer[RX_BUFFER_SIZE];
    
    for(i = 0; buffer[i] != '\0'; i++) {
        temp_buffer[i] = buffer[i];
    }
    temp_buffer[i] = '\0';
    
    if((temp_buffer[0] > '9' || temp_buffer[0] < '0') && temp_buffer[0] != '-' && temp_buffer[0] != '+') {
        return 0.0;
    }
    
    i = 0;
    if(temp_buffer[0] == '-') {
        i = 1;
        flag = 1;
    } else if(temp_buffer[0] == '+') {
        i = 1;
    }
    
    for(tens = 1; temp_buffer[i] != '.' && temp_buffer[i] != '\0'; i++, tens *= 10);
    if(temp_buffer[i] == '\0')
        return 0.0;
    tens /= 10;
    
    for(i = flag, ret = 0; temp_buffer[i] != '.' && temp_buffer[i] != '\0'; i++, tens /= 10) {
        ret += (temp_buffer[i] - '0') * (float)tens;
    }
    if(temp_buffer[i] == '\0')
        return 0.0;
    
    i++;    
    for(tens = 10; temp_buffer[i] != '\0'; i++, tens *= 10) {
        ret += (temp_buffer[i] - '0') / (float)tens;
    }
    
    if(flag)
        ret *= -1;
    
    return ret;
}

void __ISR_AT_VECTOR(_UART3_RX_VECTOR, IPL6AUTO) MotorA_rx(void) {
    static unsigned int r;
    do {
        r = U3RXREG & 0xFF;
        if(r == '\n') {
            rx_buffer_A[rx_buffer_index_A] = '\0';
            rx_buffer_index_A = 0;
            rpm_A = parse_rx(rx_buffer_A);
        } else {
            rx_buffer_A[rx_buffer_index_A++] = r;
        }        
    }while(U3STAbits.URXDA);    
    
    if(U3STAbits.OERR)
        U3STAbits.OERR = 0;
    
    IFS4bits.U3RXIF = 0;
}

void __ISR_AT_VECTOR(_UART2_RX_VECTOR, IPL6AUTO) MotorB_rx(void) {
    static unsigned int r;
    do {
        r = U2RXREG & 0xFF;
        if(r == '\n') {
            rx_buffer_B[rx_buffer_index_B] = '\0';
            rx_buffer_index_B = 0;
            rpm_B = parse_rx(rx_buffer_B);
        } else {
            rx_buffer_B[rx_buffer_index_B++] = r;
        }        
    }while(U2STAbits.URXDA);    
    
    if(U2STAbits.OERR)
        U2STAbits.OERR = 0;
    
    IFS4bits.U2RXIF = 0;
}

void __ISR_AT_VECTOR(_UART4_RX_VECTOR, IPL6AUTO) MotorC_rx(void) {
    static unsigned int r;
    do {
        r = U4RXREG & 0xFF;
        if(r == '\n') {
            rx_buffer_C[rx_buffer_index_C] = '\0';
            rx_buffer_index_C = 0;
            rpm_C = parse_rx(rx_buffer_C);
        } else {
            rx_buffer_C[rx_buffer_index_C++] = r;
        }        
    }while(U4STAbits.URXDA);    
    
    if(U4STAbits.OERR)
        U4STAbits.OERR = 0;
    
    IFS5bits.U4RXIF = 0;
}

void USART1_init(unsigned long int baud_rate) {
    TRISDbits.TRISD10 = 1;
    TRISDbits.TRISD11 = 0;
    U1MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U1RXRbits.U1RXR = 0b0100;	// RF1
    RPD11Rbits.RPD11R = 0b0001;	// RD11
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
    U2MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U2RXRbits.U2RXR = 0b0111;	// RB7
    RPB6Rbits.RPB6R = 0b0010;	// RB6
    CFGCONbits.IOLOCK = 1;
    
    U2BRG = (100000000.0f / (float)baud_rate / 16.0f) - 1;
    
    U2STA = 0;
    
    U2MODE = 0;
    
    U2STAbits.URXEN = 1;
    U2STAbits.UTXEN = 1;
    
    IFS4bits.U2RXIF = 0;
    IEC4bits.U2RXIE = 1;
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
    U3RXRbits.U3RXR = 0b0001;	// RG7
    RPG8Rbits.RPG8R = 0b0001;	// RG8
    CFGCONbits.IOLOCK = 1;
    
    U3BRG = (100000000.0f / (float)baud_rate / 16.0f) - 1;
    
    U3STA = 0;
    
    U3MODE = 0;
    
    U3STAbits.URXEN = 1;
    U3STAbits.UTXEN = 1;
    
    IFS4bits.U3RXIF = 0;
    IEC4bits.U3RXIE = 1;
    IPC39bits.U3RXIP = 6;
    IPC39bits.U3RXIS = 0; 
    
    IEC4bits.U3TXIE = 0;
	IFS4bits.U3TXIF = 0;
    
    U3MODEbits.ON = 1; 
}

void USART4_init(unsigned long int baud_rate) {
    TRISBbits.TRISB14 = 1;
    TRISBbits.TRISB8 = 0;
    U4MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U4RXRbits.U4RXR = 0b0010;	// RB14
    RPB8Rbits.RPB8R = 0b0010;	// RB8
    CFGCONbits.IOLOCK = 1;
    
    U4BRG = (100000000.0f / (float)baud_rate / 16.0f) - 1;
    
    U4STA = 0;
    
    U4MODE = 0;
    
    U4STAbits.URXEN = 1;
    U4STAbits.UTXEN = 1;
    
    IFS5bits.U4RXIF = 0;
    IEC5bits.U4RXIE = 1;
    IPC42bits.U4RXIP = 6;
    IPC42bits.U4RXIS = 0; 
    
    IEC5bits.U4TXIE = 0;
    
    U4MODEbits.ON = 1; 
}

void USART5_init(unsigned long int baud_rate) {
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB10 = 1;
    U5MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U5RXRbits.U5RXR = 0b1000;	// RB5
    RPG6Rbits.RPG6R = 0b0011;	// RG6
    CFGCONbits.IOLOCK = 1;
    
    U5BRG = (100000000.0f / (float)baud_rate / 4.0f) - 1;
    
    U5STA = 0;
    
    U5MODE = 0;
    U5MODEbits.BRGH = 1;
    
    U5STAbits.URXEN = 1;
    U5STAbits.UTXEN = 1;
    
    U1STAbits.URXISEL = 2;
    
    IFS5bits.U5RXIF = 0;
    IEC5bits.U5RXIE = 1;
    IPC45bits.U5RXIP = 6;
    IPC45bits.U5RXIS = 0; 
    
    IEC5bits.U5TXIE = 0;
    
    U5MODEbits.ON = 1; 
}

void USART6_init(unsigned long int baud_rate) {
//    TRISBbits.TRISB14 = 1;
//    TRISBbits.TRISB8 = 0;
//    U6MODEbits.ON = 0;
//    CFGCONbits.IOLOCK = 0;
//    U6RXRbits.U4RXR = 0b0010;
//    RPB8Rbits.RPB8R = 0b0010;
//    CFGCONbits.IOLOCK = 1;
    
    U6BRG = (100000000.0f / (float)baud_rate / 16.0f) - 1;
    
    U6STA = 0;
    
    U6MODE = 0;
    
    U6STAbits.URXEN = 1;
    U6STAbits.UTXEN = 1;
    
    IFS5bits.U6RXIF = 0;
    IEC5bits.U6RXIE = 0;
    IPC47bits.U6RXIP = 6;
    IPC47bits.U6RXIS = 0; 
    
    IEC5bits.U6TXIE = 0;
    
    U6MODEbits.ON = 1; 
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
        case 6:
            while(U6STAbits.UTXBF);
            U6TXREG = byte;
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
    for(; tens > 0; tens /= 10) {
        USART_send(port, ((a / tens) % 10) + '0');
	}
}

void USART_write_float(unsigned char port, double a, unsigned char right) {
    unsigned char i;
    long int tens;
    long int t;

    if(a < 0) {
        a *= (-1);
        USART_send(port, '-');
    } 
    //else USART_send(port, '+');
    
    if(a >= 1.0) {
        for(tens = 1, t = a; tens <= t; tens *= 10);
        tens /= 10;
        for(; tens > 0; tens /= 10)
            USART_send(port, ((t / tens) % 10) + '0');
    } else {
        USART_send(port, '0');
    }

    USART_send(port, '.');
    for(i = 0, tens = 10; i < right; i++, tens *= 10)
        USART_send(port, ((long int)(a * tens) % 10) + 48);
}