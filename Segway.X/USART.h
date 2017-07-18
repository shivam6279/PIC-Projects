#ifndef _USART_H_
#define _USART_H_

/*
USART read and write functions. Use for XBee
Requires MCU library eg PIC16F877A.h for the register names TXSTA. TXIE etc
*/

//unsigned char recieve; //Interrupt recieve byte

void USART_init(long int baud_rate, unsigned char rxie);
void USART_send(unsigned char byte);
unsigned char receive_byte();
void USART_send_str(const char str[]);
void USART_write_int(int a, unsigned char precision);
void USART_write_float(double a, unsigned char left, unsigned char right);


void USART_init(long int baud_rate, unsigned char rxie){
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 1;
    TXSTA = 36;
    if(baud_rate == 9600){
        TXSTAbits.BRGH = 0;
        SPBRG = 77;
    }
    else if(baud_rate == 19200){
        TXSTAbits.BRGH = 1;
        SPBRG = 155;
    }
    else if(baud_rate == 38400){
        TXSTAbits.BRGH = 1;
        SPBRG = 77;
    }
    else if(baud_rate == 111111){
        TXSTAbits.BRGH = 1;
        SPBRG = 26;
    }
    else if(baud_rate == 115200){
        TXSTAbits.BRGH = 1;
        SPBRG = 25;
    }
    RCSTA = 144;
    TXIE = 0;
    if(rxie == 1){
        RCIE = 1;
        GIE = 1;
        PEIE = 1;
    }
    else{
        RCIE = 0;
    }
}

void USART_send(unsigned char byte){
    while(!TXIF);
    TXREG = byte;
}

unsigned char receive_byte(){
    if(OERR){
        CREN = 0;
        CREN = 1;
    }
    while(!RCIF);
    return RCREG;
}


void USART_send_str(const char str[]){
    int i;
    for(i = 0; str[i] != '\0'; i++){
        USART_send(str[i]);
    }
}

void USART_write_int(int a, unsigned char precision){
    if(a < 0){
        a *= (-1);
        USART_send('-');
    }
    else{
        USART_send('+');
    }
    if(precision >= 6) USART_send(((a / 100000) % 10) + 48);
    if(precision >= 5) USART_send(((a / 10000) % 10) + 48);
    if(precision >= 4) USART_send(((a / 1000) % 10) + 48);
    if(precision >= 3) USART_send(((a / 100) % 10) + 48);
    if(precision >= 2) USART_send(((a / 10) % 10) + 48);
    if(precision >= 1) USART_send((a % 10) + 48);
}

void USART_write_float(double a, unsigned char left, unsigned char right){
    unsigned char i;
    long int tens = 10;
    if(a < 0){
        a *= (-1);
        USART_send('-');
    }
    else{
        USART_send('+');
    }    
    if(left >= 7) USART_send(((int)(a / 1000000) % 10) + 48);
    if(left >= 6) USART_send(((int)(a / 100000) % 10) + 48);
    if(left >= 5) USART_send(((int)(a / 10000) % 10) + 48);
    if(left >= 4) USART_send(((int)(a / 1000) % 10) + 48);
    if(left >= 3) USART_send(((int)(a / 100) % 10) + 48);
    if(left >= 2) USART_send(((int)(a / 10) % 10) + 48);
    if(left >= 1) USART_send(((int)a % 10) + 48);
    USART_send('.');
    for(i = 0; i < right; i++){
        USART_send(((long int)(a * tens) % 10) + 48);
        tens *= 10;
    }
}

/*void interrupt ISR(){ // Copy into main c program
    if(RCIF){
        //receive = RCREG;//Un-comment if using recieve interrupts, if RCIE = 1
        if(OERR){
            CREN = 0;
            CREN = 1;
        }
    }
}*/

#endif