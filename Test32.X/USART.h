#ifndef _USART_H_
#define _USART_H_

/*
USART read and write functions. Use for XBee
Requires MCU library eg PIC16F877A.h for the register names TXSTA. TXIE etc
*/

//unsigned char recieve; //Interrupt recieve byte

void USART_init();
void send_byte(unsigned char byte);

void USART_init(){
    U1MODEbits.ON = 0;
    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB3 = 0;
    CFGCONbits.IOLOCK = 0;
    U1RXRbits.U1RXR = 8;//U1RX at RB5
    RPB3Rbits.RPB3R = 1;//U1TX at RB3
    CFGCONbits.IOLOCK = 1;
    
    IFS3bits.U1RXIF = 0;
    IEC3bits.U1RXIE = 1;
    IEC3bits.U1TXIE = 0;
    IPC28bits.U1RXIP = 7;
    IPC28bits.U1RXIS = 0; 
    
    U1BRG = 650;
    
    U1STAbits.ADM_EN = 0;
    U1STAbits.UTXISEL = 3;
    U1STAbits.UTXINV = 0;
    U1STAbits.UTXBRK = 0;
    U1STAbits.URXISEL = 0;
    U1STAbits.OERR = 0;
    
    U1MODEbits.SIDL = 0;
    U1MODEbits.IREN = 0;
    U1MODEbits.UEN = 0;
    U1MODEbits.WAKE = 1;
    U1MODEbits.LPBACK = 0;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.RXINV = 0;//1;
    U1MODEbits.BRGH = 0;
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;
    
    U1MODEbits.ON = 1; 
    U1STAbits.URXEN = 1;
    U1STAbits.UTXEN = 1;
}

void send_byte(unsigned char byte){
    while(U1STAbits.UTXBF);
    U1TXREG = byte;
}

/*void interrupt ISR(){ // Copy into main c program
    if(RCIF){
        //receive = RCREG;//Un-comment if using recieve interrupts, if RCIE = 1
        if(U1STAbits.OERR){
            U1STAbits.OERR = 0;
            CREN = 0;
            CREN = 1;
        }
    }
}*/

#endif