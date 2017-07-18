#ifndef _GLCD_H_
#define _GLCD_H_

/*
Graphics LCD library
requires xc.h
*/

#define RS  RB7
#define RW RB6
#define EN RB5
#define DB0 RB4
#define DB1 RB3
#define DB2 RB2
#define DB3 RB1
#define DB4 RB0
#define DB5 RD7
#define DB6 RD6
#define DB7 RD5
#define CS1 RD4
#define CS2 RC5
#define RST RC4

#define TB0 TRISB4
#define TB1 TRISB3
#define TB2 TRISB2
#define TB3 TRISB1
#define TB4 TRISB0
#define TB5 TRISD7
#define TB6 TRISD6
#define TB7 TRISD5

void GLCD_init();
unsigned int pow(unsigned int base, unsigned int exp);
void data_write(unsigned char a);
void set_x(unsigned int x);
void set_y(unsigned int y);
void clear_screen();
void dot(unsigned int x, unsigned int y, unsigned int c);

void GLCD_init(){
    __delay_us(10);
    CS1 = 1;
    CS2 = 1;
    RS = 0;
    RW = 0;
    data_write(63);
    RS = 1;
    RW = 0;
    data_write(192);
    clear_screen();
}

unsigned int pow(unsigned int base, unsigned int exp){
    unsigned int i,ret = 1;
    for(i = 1;i <= exp;i++){
        ret *= base;
    }
    return ret;
}

void data_write(unsigned char a){
    unsigned char bin[8] = {0,0,0,0,0,0,0,0},i;
    for(i=0;i<8;i++){
        bin[i] = a%2;
        a /= 2;
    }
    DB0 = bin[0];
    DB1 = bin[1];
    DB2 = bin[2];
    DB3 = bin[3];
    DB4 = bin[4];
    DB5 = bin[5];
    DB6 = bin[6];
    DB7 = bin[7];
    EN = 1;
    __delay_us(10);
    EN = 0;
}

void set_x(unsigned int x){
    unsigned int t;
    if(x < 64){
        CS1 = 1;
        CS2 = 0;
        t = x;
    }
    else{
        CS1 = 0;
        CS2 = 1;
        t = x - 64;
    }
    RS = 0;
    RW = 0;
    data_write(64 + t);
}

void set_y(unsigned int y){
    CS1 = 1;
    CS2 = 1;
    RS = 0;
    RW = 0;
    data_write(184 + y);
}

void clear_screen(){
    unsigned int i,j;
    for(i = 0;i < 8;i++){
        set_y(i);
        set_x(0);
        RS = 1;
        RW = 0;
        for(j = 0;j < 128;j++){
            set_x(j);
            RS = 1;
            RW = 0;
            data_write(0);
        }
    }
}

void dot(unsigned int x, unsigned int y, unsigned int c){
    unsigned int temp;
    set_y(y/8);
    set_x(x);
    RS = 1;
    RW = 1;
    TB0 = 1;
    TB1 = 1;
    TB2 = 1;
    TB3 = 1;
    TB4 = 1;
    TB5 = 1;
    TB6 = 1;
    TB7 = 1;
    EN = 1;
    __delay_us(10);
    EN = 0;
    EN = 1;
    __delay_us(10);
    EN = 0;
    temp = DB7 * 128 + DB6 * 64 + DB5 * 32 + DB4 * 16 + DB3 * 8 + DB2 * 4 + DB1 * 2 + DB0;
    TB0 = 0;
    TB1 = 0;
    TB2 = 0;
    TB3 = 0;
    TB4 = 0;
    TB5 = 0;
    TB6 = 0;
    TB7 = 0;
    set_y(y / 8);
    set_x(x);
    RS = 1;
    RW = 0;
    if(c == 1){
        data_write(temp | pow(2, (y % 8)));
    }
    else if(c == 0){
        data_write(temp & (255 - pow(2, (y % 8))));
    }
}

#endif
