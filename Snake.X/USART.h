#ifndef _USART_H_
#define _USART_H_

/*
USART read and write functions. Use for XBee
Requires MCU library eg PIC16F877A.h for the register names TXSTA. TXIE etc
*/

//unsigned char recieve; //Interrupt recieve byte

void USART_init(int baud_rate, unsigned char rxie);
void send_byte(unsigned char byte);
unsigned char receive_byte();


void USART_init(int baud_rate, unsigned char rxie){
    float speed;
    speed = ((_XTAL_FREQ / 16) / baud_rate) - 1;
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    TXSTA = 36;
    SPBRG = 20;
    RCSTA = 144;
    TXIE = 0;
    RCIE = rxie;
}

void send_byte(unsigned char byte){
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

/*void interrupt ISR(){ // Copy into main c program
    if(RCIF){
        //receive = RCREG;//Un-comment if using receive interrupts, if RCIE = 1
        if(OERR){
            CREN = 0;
            CREN = 1;
        }
    }
}*/

#endif