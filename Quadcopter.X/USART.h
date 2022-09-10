#ifndef _USART_H_
#define _USART_H_

#include <xc.h>

#define XBEE_TX_INTERRUPT IEC5bits.U4TXIE

extern void USART4_init(unsigned long int);
extern void USART4_send(unsigned char byte);
extern void USART4_send_str(char[]);
extern void USART4_write_int(int);
extern void USART4_write_float(double, unsigned char);

extern void USART3_init(unsigned long int);
extern void USART5_init(unsigned long int);


#endif