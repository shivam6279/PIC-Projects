#ifndef _USART_H_
#define _USART_H_

#define RX_BUFFER_SIZE 1024

extern void USART4_init(unsigned long int baud_rate);
extern void USART4_send(unsigned char byte);
extern void USART4_send_str(char str[]);
extern void USART4_write_int(long int a);
extern void USART4_write_float(double a, unsigned char right);

extern volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
extern volatile unsigned char rx_rdy;
extern volatile unsigned char play_tone;

#endif