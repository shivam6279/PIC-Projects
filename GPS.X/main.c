#define _XTAL_FREQ 48000000
#include <xc.h>
#include <pic18f4550.h>
#include <math.h>
#include <ctype.h>
#include "I2C.h"
#include "USART.h"

#pragma config PLLDIV = 12
#pragma config CPUDIV = OSC1_PLL2
#pragma config USBDIV = 1
#pragma config FOSC = HS
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRT = OFF
#pragma config BOR = OFF
#pragma config BORV = 2
#pragma config VREGEN = OFF
#pragma config WDT = OFF
#pragma config WDTPS = 32768
#pragma config CCP2MX = ON
#pragma config PBADEN = OFF
#pragma config LPT1OSC = OFF
#pragma config MCLRE = ON
#pragma config STVREN = OFF
#pragma config LVP = OFF
#pragma config ICPRT = OFF
#pragma config XINST = OFF
#pragma config CP0 = OFF
#pragma config CP1 = OFF
#pragma config CP2 = OFF
#pragma config CP3 = OFF
#pragma config CPB = OFF
#pragma config CPD = OFF
#pragma config WRT0 = OFF
#pragma config WRT1 = OFF
#pragma config WRT2 = OFF
#pragma config WRT3 = OFF
#pragma config WRTC = OFF
#pragma config WRTB = OFF
#pragma config WRTD = OFF
#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF
#pragma config EBTRB = OFF

#define led_red PORTBbits.RB2
#define led_blue PORTBbits.RB3

void delay_ms(int x){
    int i;
    for(i = 0; i < x; i++){
        __delay_ms(1);
    }
}

void delay_us(int x){
    int i;
    for(i = 0; i < x; i++){
        __delay_us(1);
    }
}

void timer_init(){
    T0CS = 0;
    T0SE = 0;
    PSA = 0;
    T0PS2 = 1;
    T0PS1 = 0;
    T0PS0 = 1;
    IPEN = 0;
    GIE = 1;
    TMR0 = 0;
    TMR0IE = 1;
    TMR0ON = 1;
}

void init(){
    TRISA = 1;
    TRISB = 0;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    ADCON0 = 0;
    ADCON1 = 15;
}

void interrupt ISR(){
    if(RCIF){
        if(OERR){
            CREN = 0;
            CREN = 1;
        }
    }
    if(TMR0IF){
        TMR0IF = 0;
    }
}

long int pow_10(int a){
  long int t;
  int i;
  for(i = 0, t = 1; i < a; i++){
    t *= 10;
  }
  return t;
}

double GPS_StringToFloat(char *str){
  double temp = 0, temp_decimal = 0;
  int i = 0, j;
  long int big;
  while(*str != '.'){
    str++;
    i++;
  }
  str -= 3;
  i -= 3;
  for(big = 1; i >= 0; i--, str--, big *= 10) temp += (*str - 48) * big;
  while(*str != '.') str++;
  str -= 2;
  i = 0;
  while(*str != '\0'){
    str++;
    i++;
  }
  i--;
  j = i;
  str--;
  for(big = 1; i >= 0; i--, str--){
    if(*str != '.'){
      temp_decimal += (*str - 48) * big;
      big *= 10;
    }
  }
  temp_decimal = temp_decimal * 10 / 6;
  temp += (temp_decimal / pow_10(j));
  return temp;
}

void main(){
    float latitude, longitude;
    unsigned char i;
    char ch, status, str_latitude[12], str_longitude[12];
    init();
    USART_init(9600, 0);
    i2c_init();
    led_red = 0;
    led_blue = 0;
    while(1){
        ch = receive_byte();
        if(ch == '$'){
            ch = receive_byte();
            if(ch == 'G'){
                ch = receive_byte();
                if(ch == 'P'){
                    ch = receive_byte();
                    if(ch == 'R'){
                        ch = receive_byte();
                        if(ch == 'M'){
                            ch = receive_byte();
                            if(ch == 'C'){
                                ch = receive_byte(); 
                                do{
                                    ch = receive_byte();
                                }while(ch != ',');
                                status = receive_byte();
                                send_byte(status);
                                send_byte('\n');
                                if(status == 'V'){
                                    led_blue = 0;
                                    led_red = 1;
                                    latitude = 0;
                                    longitude = 0;
                                }
                                else{
                                    led_blue = 1;
                                    led_red = 0;
                                    ch = receive_byte(); 
                                    for(i = 0;;i++){
                                        ch = receive_byte(); 
                                        if(ch == ','){
                                            break;
                                        }
                                        str_latitude[i] = ch;
                                    }
                                    str_latitude[i] = '\0';
                                    ch = receive_byte(); 
                                    ch = receive_byte(); 
                                    for(i = 0;;i++){
                                        ch = receive_byte(); 
                                        if(ch == ','){
                                            break;
                                        }
                                        str_longitude[i] = ch;
                                    }
                                    str_longitude[i] = '\0';
                                    latitude = GPS_StringToFloat(str_latitude);
                                    longitude = GPS_StringToFloat(str_longitude);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}