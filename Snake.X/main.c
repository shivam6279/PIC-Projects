#define _XTAL_FREQ 20000000
#include <xc.h>
#include <pic16f877a.h>
#include <stdlib.h>
#include <stdio.h>
#include "GLCD.h"
#include "USART.h"

#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config BOREN = OFF
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

unsigned char receive;

void G(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(127 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b01001110);
    data_write(0b10001001);
    data_write(0b10001001);
    data_write(0b10000001);
    data_write(0b01111110);
    data_write(0b00000000);
}

void A(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(127 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b01111111);
    data_write(0b11001000);
    data_write(0b10001000);
    data_write(0b11001000);
    data_write(0b01111111);
    data_write(0b00000000);
}

void M(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(127 - x * 6);
    RS = 1;
    RW = 0;    
    data_write(0b11111111);
    data_write(0b01100000);
    data_write(0b00110000);
    data_write(0b01100000);
    data_write(0b11111111);
    data_write(0b00000000);
}

void E(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(127 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b10000001);
    data_write(0b10001001);
    data_write(0b10001001);
    data_write(0b10001001);
    data_write(0b11111111);
    data_write(0b00000000);
}

void O(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(127 - x * 6);
    RS = 1;
    RW = 0;    
    data_write(0b01111110);
    data_write(0b10000001);
    data_write(0b10000001);
    data_write(0b10000001);
    data_write(0b01111110);
    data_write(0b00000000);
}

void V(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(127 - x * 6);
    RS = 1;
    RW = 0;    
    data_write(0b11111100);
    data_write(0b00000110);
    data_write(0b00000011);
    data_write(0b00000110);
    data_write(0b11111100);
    data_write(0b00000000);
}

void P(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(127 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b01110000);
    data_write(0b10001000);
    data_write(0b10001000);
    data_write(0b10001000);
    data_write(0b11111111);
    data_write(0b00000000);
}

void R(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(127 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b01110001);
    data_write(0b10001010);
    data_write(0b10001100);
    data_write(0b10001000);
    data_write(0b11111111);
    data_write(0b00000000);
}

void S(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(127 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b01000110);
    data_write(0b10001001);
    data_write(0b10001001);
    data_write(0b10001001);
    data_write(0b01110010);
    data_write(0b00000000);
}

void T(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(127 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b10000000);
    data_write(0b10000000);
    data_write(0b11111111);
    data_write(0b10000000);
    data_write(0b10000000);
    data_write(0b00000000);
}

void bracket_close(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(127 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b00000000);
    data_write(0b00111100);
    data_write(0b01000010);
    data_write(0b10000001);
    data_write(0b00000000);
}

void bracket_open(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(127 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b10000001);
    data_write(0b01000010);
    data_write(0b00111100);
    data_write(0b00000000);
    data_write(0b00000000);
    data_write(0b00000000);
}


void lose(){
    G(6, 0);
    A(7, 0);
    M(8, 0);
    E(9, 0);
    O(6, 2);
    V(7, 2);
    E(8, 2);
    R(9, 2);
    P(2, 4);
    R(3, 4);
    E(4, 4);
    S(5, 4);
    S(6, 4);
    bracket_open(7, 4);
    A(8, 4);
    bracket_close(9, 4);
    T(4, 6);
    O(5, 6);
    R(2, 7);
    E(3, 7);
    S(4, 7);
    T(5, 7);
    A(6, 7);
    R(7, 7);
    T(8, 7);
}

void init(){
    TRISB = 0;
    TRISC = 137;
    TRISD = 15;
    GLCD_init();
    srand(TMR0);
    USART_init(0, 1);
}

void interrupt ISR(){
    if(RCIF){
        receive = RCREG;
        if(OERR){
            CREN = 0;
            CREN = 1;
        }
    }
}

void main(){
    RST = 1;
    receive = 0;
    init();
    unsigned int rx, ry;
    unsigned char brk = 0, apple = 0, i, temp_a = 1, temp_b = 1;
    signed char x, y, body[48][2], l, d, difficulty = 2;
    while(1){
        clear_screen();
        l = 10;
        for(i = 0; i < l; i++){
            body[i][0] = 68 - i;
            body[i][1] = 32;
        }
        d = 1;
        for(i = 0; i < l; i++){
            dot(body[i][0], body[i][1], 1);
        }
        while(1){
            if(apple == 0){
                apple = 1;
                rx = rand() % 128;
                ry = rand() % 64;
                dot(rx, ry, 1);
            }
            if(body[0][0] == rx && body[0][1] == ry){
                l++;
                body[l - 1][0] = body[l - 2][0];
                body[l - 1][1] = body[l - 2][1];
                apple = 0;
            }
            if(receive){
                if((receive % 16) == 3 && d != 3) d = 1;
                else if((receive % 16) == 5 && d != 4) d = 2;
                else if((receive % 16) == 7 && d != 1) d = 3;
                else if((receive % 16) == 1 && d != 2) d = 4;
            }
            else{
                if(RD0 == 0 && d != 3) d = 1;
                else if(RC0 == 0 && d != 4) d = 2;
                else if(RC3 == 0 && d != 1) d = 3;
                else if(RD1 == 0 && d != 2) d = 4;
            }
            if(d == 1){
                x = 1;
                y = 0;
            }
            else if(d == 2){
                x = 0;
                y = 1;
            }
            else if(d == 3){
                x = -1;
                y = 0;
            }
            else if(d == 4){
                x = 0;
                y = -1;
            }
            if(RD3 == 0 && temp_a == 1){
                temp_a = 0;
                if(++difficulty > 3)difficulty = 3;
            }
            else temp_a = 1;
            if(RD2 == 0 && temp_b == 1){
                temp_b = 0;
                if(--difficulty < 0)difficulty = 0;
            }
            else temp_b = 1;
            if(difficulty == 0) __delay_ms(100);
            else if(difficulty == 1) __delay_ms(75);
            else if(difficulty == 2) __delay_ms(50);
            else if(difficulty == 3) __delay_ms(25);
            for(i = l - 1; i > 0; i--){
                body[i][0] = body[i - 1][0];
                body[i][1] = body[i - 1][1];
            }
            body[0][0] += x;
            body[0][1] += y;
            for(i = 1; i < l; i++){
                if(body[0][0] == body[i][0] && body[0][1] == body[i][1]){
                    brk = 1;
                    break;
                }
            }
            if(brk == 1){
                brk = 0;
                break;
            }
            if(body[0][0] == 127){
                body[0][0] = 0;
            }
            else if(body[0][0] == -1){
                    body[0][0] = 127;
            }
            if(body[0][1] == 64){
                body[0][1] = 0;
            }
            else if(body[0][1] == -1){
                body[0][1] = 63;
            }
            dot(body[0][0],body[0][1],1);
            dot(body[l - 1][0],body[l - 1][1],0);
        }
        __delay_ms(1000);
        for(i = 0; i < l; i++){
            dot(body[i][0],body[i][1],0);
        }
        apple = 0;
        lose();
        while(RD3 == 1);
    }
}
