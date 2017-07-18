#ifndef _USART_H_
#define _USART_H_

void USART1_init(unsigned long int baud_rate);
void USART1_send(unsigned char byte);
void USART1_send_str(char str[]);
void USART1_write_int(int a, unsigned char precision);
void USART1_write_float(double a, unsigned char left, unsigned char right);
        
void USART5_init(unsigned long int baud_rate);

void USART1_init(unsigned long int baud_rate){
    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB3 = 0;
    U1MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U1RXRbits.U1RXR = 8;//U1RX at RB5
    RPB3Rbits.RPB3R = 1;//U1TX at RB3
    CFGCONbits.IOLOCK = 1;
    
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

void USART1_send(unsigned char byte){
    while(U1STAbits.UTXBF);
    U1TXREG = byte;
}
void USART1_send_str(char str[]){
    int i;
    for(i = 0; str[i] != '\0'; i++){
        USART1_send(str[i]);
    }
}

void USART1_write_int(int a, unsigned char precision){
    if(a < 0){
        a *= (-1);
        USART1_send('-');
    }
    else{
        USART1_send('+');
    }
    if(precision >= 6) USART1_send(((a / 100000) % 10) + 48);
    if(precision >= 5) USART1_send(((a / 10000) % 10) + 48);
    if(precision >= 4) USART1_send(((a / 1000) % 10) + 48);
    if(precision >= 3) USART1_send(((a / 100) % 10) + 48);
    if(precision >= 2) USART1_send(((a / 10) % 10) + 48);
    if(precision >= 1) USART1_send((a % 10) + 48);
}

void USART1_write_float(double a, unsigned char left, unsigned char right){
    unsigned char i;
    long int tens = 10;
    if(a < 0){
        a *= (-1);
        USART1_send('-');
    }
    else{
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
    for(i = 0; i < right; i++){
        USART1_send(((long int)(a * tens) % 10) + 48);
        tens *= 10;
    }
}

void USART5_init(unsigned long int baud_rate){
    TRISCbits.TRISC14 = 1;
    U5MODEbits.ON = 0;
    CFGCONbits.IOLOCK = 0;
    U5RXRbits.U5RXR = 7;//U5RX at RC14
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

#endif