#ifndef _USART_H_
#define _USART_H_

extern void USART1_init(unsigned long int baud_rate);
extern void USART1_send(unsigned char byte);
extern void USART1_send_str(char str[]);
extern void USART1_write_int(int a, unsigned char precision);
extern void USART1_write_float(double a, unsigned char left, unsigned char right);
        
extern void USART5_init(unsigned long int baud_rate);

#endif