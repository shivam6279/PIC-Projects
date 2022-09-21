#ifndef _USART_H_
#define _USART_H_

#define UART1_TX_INTERRUPT IEC3bits.U1TXIE

#define RX_BUFFER_SIZE 1024

extern void USART1_init(unsigned long int);
extern void USART2_init(unsigned long int);
extern void USART3_init(unsigned long int);
extern void USART4_init(unsigned long int);
extern void USART5_init(unsigned long int);
extern void USART6_init(unsigned long int);

extern void USART_send(unsigned char, unsigned char);
extern void USART_send_str(unsigned char, char[]);
extern void USART_write_int(unsigned char, int);
extern void USART_write_float(unsigned char, double, unsigned char);

extern float parse_rx(volatile char*);

extern volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
extern volatile unsigned char rx_rdy;

extern volatile float rpm_A, rpm_B, rpm_C;

#endif