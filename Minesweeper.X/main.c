#define _XTAL_FREQ 20000000
#include <xc.h>
#include <pic16f877a.h>
#include <stdlib.h>
#include <stdio.h>
#include "USART.h"
#include "GLCD.h"

#pragma config FOSC = HS     
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config BOREN = OFF
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

unsigned char board[8][8], open[8][8];
int x = 0, y = 0;
unsigned char  count = 0, button_count = 0, recieve;

void one(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b00000000);
    if(x == 10) set_x(64);
    data_write(0b00000001);
    data_write(0b11111111);
    data_write(0b01000001);
    data_write(0b00000000);
}

void two(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b01110001);
    if(x == 10) set_x(64);
    data_write(0b10001001);
    data_write(0b10001001);
    data_write(0b10000101);
    data_write(0b10000011);
}

void three(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b01101110);
    if(x == 10) set_x(64);
    data_write(0b10010001);
    data_write(0b10010001);
    data_write(0b10010001);
    data_write(0b01000010);
}

void four(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b00000100);
    if(x == 10) set_x(64);
    data_write(0b11111111);
    data_write(0b01000100);
    data_write(0b00110100);
    data_write(0b00001100);
}

void five(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b01001110);
    if(x == 10) set_x(64);
    data_write(0b10010001);
    data_write(0b10010001);
    data_write(0b10010001);
    data_write(0b11110010);
}

void six(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b01000110);
    if(x == 10) set_x(64);
    data_write(0b10001001);
    data_write(0b10001001);
    data_write(0b10001001);
    data_write(0b01111110);
}

void seven(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b11000000);
    if(x == 10) set_x(64);
    data_write(0b10000111);
    data_write(0b10001000);
    data_write(0b10010000);
    data_write(0b11100000);
}

void eight(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b01110110);
    if(x == 10) set_x(64);
    data_write(0b10001001);
    data_write(0b10001001);
    data_write(0b10001001);
    data_write(0b01101110);
}

void blank(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b00000000);
    if(x == 10) set_x(64);
    data_write(0b00000000);
    data_write(0b00000000);
    data_write(0b00000000);
    data_write(0b00000000);
}

void closed(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b11111111);
    if(x == 10) set_x(64);
    data_write(0b11111111);
    data_write(0b11111111);
    data_write(0b11111111);
    data_write(0b11111111);
}

void mine(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b01000001);
    if(x == 10) set_x(64);
    data_write(0b00010011);
    data_write(0b10000111);
    data_write(0b00010011);
    data_write(0b01000001);
}

void bar(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b00000000);
    if(x == 10) set_x(64);
    data_write(0b00000000);
    data_write(0b11111111);
    data_write(0b00000000);
    data_write(0b00000000);
}

void flag(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b00100001);
    if(x == 10) set_x(64);
    data_write(0b01110001);
    data_write(0b11111111);
    data_write(0b00000001);
    data_write(0b00000001);
}

void box(unsigned char x, unsigned char y){
    set_y(7 - y);
    set_x(122 - x * 6);
    RS = 1;
    RW = 0;
    data_write(0b00000000);
    data_write(0b11111111);
    if(x == 10) set_x(64);
    data_write(0b10000001);
    data_write(0b10000001);
    data_write(0b10000001);
    data_write(0b11111111);
}

void lose(){
    while(1);
}

void reset(){
    unsigned char i,r1,r2;
    for(i = 0; i < 64; i++){
        board[i / 8][i % 8] = 0;
        open[i / 8][i % 8] = 0;
    }
    for(i = 0; i < 10; i++){
        r1 = rand() % 8;
        r2 = rand() % 8;
        if(board[r1][r2]==10){
            i--;
        }
        else{
            board[r1][r2] = 10;
        }
    }
}

void display(){
    unsigned char i,j;
    for(i = 0; i < 8; i++){
        for(j = 0; j < 8; j++){
            bar((2 * j), i);
            if(i == y && j == x && count > 20){
                if(board[i][j] == 0 && open[i][j] == 1) box((2 * j + 1), i);
                else blank((2 * j + 1), i);
            }
            else{
                if(open[i][j] == 1){
                    if(board[i][j] == 0)blank((2 * j + 1), i);
                    else if(board[i][j] == 1)one((2 * j + 1), i);
                    else if(board[i][j] == 2)two((2 * j + 1), i);
                    else if(board[i][j] == 3)three((2 * j + 1), i);
                    else if(board[i][j] == 4)four((2 * j + 1), i);
                    else if(board[i][j] == 5)five((2 * j + 1), i);
                    else if(board[i][j] == 6)six((2 * j + 1), i);
                    else if(board[i][j] == 7)seven((2 * j + 1), i);
                    else if(board[i][j] == 8)eight((2 * j + 1), i);
                    else if(board[i][j] == 10)mine((2 * j + 1), i);
                }
                else if(open[i][j]==3){
                    flag((2*j + 1),i);
                }
                else{
                    closed((2*j + 1),i);
                }
            }
        }
        bar(16,i);
    }
}

void number(){
    int i,c;
    for(i = 0;i<64;i++){
        c = 0;
        if(board[i/8][i%8] != 10){
            if(board[i / 8 + 1][i % 8 + 1] == 10 && (i / 8 + 1) < 8 && (i % 8 + 1) < 8) c++;
            if(board[i / 8 + 1][i % 8] == 10 && (i / 8 + 1) < 8) c++;
            if(board[i / 8 + 1][i % 8 - 1] == 10 && (i / 8 + 1) < 8 && (i % 8 - 1) >= 0) c++;
            if(board[i / 8][i % 8 + 1] == 10 && (i % 8 + 1) < 8) c++;
            if(board[i / 8][i % 8 - 1] == 10 && (i % 8 - 1) >= 0) c++;
            if(board[i / 8 - 1][i % 8 + 1] == 10 && (i / 8 - 1) >= 0 && (i % 8 + 1) < 8) c++;
            if(board[i / 8 - 1][i % 8] == 10 && (i / 8 - 1) >= 0) c++;
            if(board[i / 8 - 1][i % 8 - 1] == 10 && (i / 8 - 1) >= 0 && (i % 8 - 1) >= 0) c++;
            board[i / 8][i % 8] = c;
        }
    }
}

void debug(){
    unsigned char i;
    for(i = 0; i < 64; i++){
        open[i / 8][i % 8] = 1;
    }
}

void interrupt ISR(){
    if(RCIF){
        recieve = RCREG;
        if(OERR){
            CREN = 0;
            CREN = 1;
        }
    }
    if(TMR0IF){
        TMR0IF = 0;
        if(++count > 40) count = 0;
        if(button_count < 20) button_count++;
    }
}

void init(){
    TRISB = 0;
    TRISC = 137;
    TRISD = 15;
    __delay_us(10);
    GLCD_init();
    srand(TMR0);
    USART_init(0, 0);
    T0CS = 0;
    T0SE = 0;
    PSA = 0;
    PS2 = 1;
    PS1 = 1;
    PS0 = 1;
    GIE = 1;
    TMR0IE = 1; 
}

void main(){
    RST = 1;
    init();
    reset();
    number();
    unsigned char i, flag;
    while(1){
        display();
        if(RC0 == 0 && button_count >= 20){
            y--;
            button_count = 0;
        }
        else if(RD0 == 0 && button_count >= 20){
            x--;
            button_count = 0;
        }
        else if(RD1 == 0 && button_count >= 20){
            y++;
            button_count = 0;
        }
        else if(RC3 == 0 && button_count >= 20){
            x++;
            button_count = 0;
        }
        if(x < 0) x = 0;
        else if(x > 7) x = 7;
        if(y < 0) y = 0;
        else if(y > 7) y = 7;
        if(RD3 == 0){
            open[y][x] = 1;
            button_count = 0;
            if(board[y][x] == 10){
                lose();
            }
        }
        else if(RD2 == 0){
            button_count = 0;
            if(open[y][x] != 1){
                if(open[y][x] == 3){
                    open[y][x] = 0;
                }
                else{
                    open[y][x] = 3;
                }
            }
        }
        flag = 1;
        for(i = 0; i < 64; i++){
            if(board[i / 8][i % 8] != 10 && open[i / 8][i % 8] == 0){
                flag = 0;
            }
        }
        if(flag == 1){
            //win();
            while(1);
        }
    }
}