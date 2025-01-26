#ifndef _USART_H_
#define _USART_H_

#include <inttypes.h>

#define RX_BUFFER_SIZE 1024

extern void USART3_init(unsigned long int);
extern void USART3_send(unsigned char);
extern void USART3_send_str(const char*);
extern void USART3_write_int(int64_t);
extern void USART3_write_float(double, unsigned char);

extern volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
extern volatile unsigned char rx_rdy;
extern volatile unsigned char play_tone;
extern volatile unsigned char auto_stop;

#endif
