#ifndef _USART_H_
#define _USART_H_

#define UART1_TX_INTERRUPT IEC3bits.U1TXIE

extern void USART1_init(unsigned long int);
extern void USART1_send(unsigned char byte);
extern void USART1_send_str(char[]);
extern void USART1_write_int(int);
extern void USART1_write_float(double, unsigned char);

extern void USART3_init(unsigned long int);

extern void USART5_init(unsigned long int);

#if board_version == 1 || board_version == 2 || board_version == 3
	extern void USART5_init(unsigned long int baud_rate);
#elif board_version == 4
	extern void USART3_init(unsigned long int baud_rate);
#endif

#endif