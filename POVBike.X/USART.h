#ifndef _USART_H_
#define _USART_H_

#define UART1_TX_INTERRUPT IEC3bits.U1TXIE

extern void USART1_init(unsigned long int baud_rate);
extern void USART1_send(unsigned char byte);
extern void USART1_send_str(char str[]);
extern void USART1_write_int(int a);
extern void USART1_write_float(double a, unsigned char right);

extern void USART3_init(unsigned long int baud_rate);
extern void USART3_send(unsigned char byte);
extern void USART3_send_str(char str[]);
extern void USART3_write_int(int a);
extern void USART3_write_float(double a, unsigned char right);

#endif