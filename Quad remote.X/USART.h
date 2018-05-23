#ifndef _USART_H_
#define _USART_H_

extern void USART1_init(unsigned long int);
extern void USART1_send(unsigned char);
extern void USART1_send_str(char str[]);
extern void USART1_write_int(int, unsigned char);
extern void USART1_write_float(double, unsigned char, unsigned char);
        
extern void USART5_init(unsigned long int);
extern void USART5_send(unsigned char);

#endif