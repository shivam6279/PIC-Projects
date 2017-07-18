#define _XTAL_FREQ 48000000
#include <pic18f4550.h>
#include <xc.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#pragma config PLLDIV = 5
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

#define RST PORTDbits.RD3
#define DC PORTDbits.RD2
#define CS PORTDbits.RD1

const unsigned char characters[][5]={
    {0x00,0x00,0x00,0x00,0x00},//  
    {0x00,0x5F,0x00,0x00,0x00},//!
    {0x00,0x07,0x00,0x07,0x00},//"
    {0x14,0x7F,0x14,0x7F,0x14},//#
    {0x24,0x2A,0x7F,0x2A,0x12},//$
    {0x23,0x13,0x08,0x64,0x62},//%
    {0x36,0x49,0x55,0x22,0x50},//&
    {0x00,0x05,0x03,0x00,0x00},//'
    {0x1C,0x22,0x41,0x00,0x00},//(
    {0x41,0x22,0x1C,0x00,0x00},//)
    {0x08,0x2A,0x1C,0x2A,0x08},//*
    {0x08,0x08,0x3E,0x08,0x08},//+
    {0xA0,0x60,0x00,0x00,0x00},//,
    {0x08,0x08,0x08,0x08,0x08},//-
    {0x60,0x60,0x00,0x00,0x00},//.
    {0x20,0x10,0x08,0x04,0x02},///
    {0x3E,0x51,0x49,0x45,0x3E},//0
    {0x00,0x42,0x7F,0x40,0x00},//1
    {0x62,0x51,0x49,0x49,0x46},//2
    {0x22,0x41,0x49,0x49,0x36},//3
    {0x18,0x14,0x12,0x7F,0x10},//4
    {0x27,0x45,0x45,0x45,0x39},//5
    {0x3C,0x4A,0x49,0x49,0x30},//6
    {0x01,0x71,0x09,0x05,0x03},//7
    {0x36,0x49,0x49,0x49,0x36},//8
    {0x06,0x49,0x49,0x29,0x1E},//9
    {0x00,0x36,0x36,0x00,0x00},//:
    {0x00,0xAC,0x6C,0x00,0x00},//;
    {0x08,0x14,0x22,0x41,0x00},//>
    {0x14,0x14,0x14,0x14,0x14},//=
    {0x41,0x22,0x14,0x08,0x00},//>
    {0x02,0x01,0x51,0x09,0x06},//?
    {0x32,0x49,0x79,0x41,0x3E},//@
    {0x7E,0x09,0x09,0x09,0x7E},//A
    {0x7F,0x49,0x49,0x49,0x36},//B
    {0x3E,0x41,0x41,0x41,0x22},//C
    {0x7F,0x41,0x41,0x22,0x1C},//D
    {0x7F,0x49,0x49,0x49,0x41},//E
    {0x7F,0x09,0x09,0x09,0x01},//F
    {0x3E,0x41,0x41,0x51,0x72},//G
    {0x7F,0x08,0x08,0x08,0x7F},//H
    {0x41,0x7F,0x41,0x00,0x00},//I
    {0x20,0x40,0x41,0x3F,0x01},//J
    {0x7F,0x08,0x14,0x22,0x41},//K
    {0x7F,0x40,0x40,0x40,0x40},//L
    {0x7F,0x02,0x0C,0x02,0x7F},//M
    {0x7F,0x04,0x08,0x10,0x7F},//N
    {0x3E,0x41,0x41,0x41,0x3E},//O
    {0x7F,0x09,0x09,0x09,0x06},//P
    {0x3E,0x41,0x51,0x21,0x5E},//Q
    {0x7F,0x09,0x19,0x29,0x46},//R
    {0x26,0x49,0x49,0x49,0x32},//S
    {0x01,0x01,0x7F,0x01,0x01},//T
    {0x3F,0x40,0x40,0x40,0x3F},//U
    {0x1F,0x20,0x40,0x20,0x1F},//V
    {0x3F,0x40,0x38,0x40,0x3F},//W
    {0x63,0x14,0x08,0x14,0x63},//X
    {0x03,0x04,0x78,0x04,0x03},//Y
    {0x61,0x51,0x49,0x45,0x43},//Z
    {0x7F,0x41,0x41,0x00,0x00},//[
    {0x02,0x04,0x08,0x10,0x20},//\'
    {0x41,0x41,0x7F,0x00,0x00},//]
    {0x04,0x02,0x01,0x02,0x04},//^
    {0x80,0x80,0x80,0x80,0x80},//_
    {0x01,0x02,0x04,0x00,0x00},//`
    {0x20,0x54,0x54,0x54,0x78},//a
    {0x7F,0x48,0x44,0x44,0x38},//b
    {0x38,0x44,0x44,0x28,0x00},//c
    {0x38,0x44,0x44,0x48,0x7F},//d
    {0x38,0x54,0x54,0x54,0x18},//e
    {0x08,0x7E,0x09,0x02,0x00},//f
    {0x18,0xA4,0xA4,0xA4,0x7C},//g
    {0x7F,0x08,0x04,0x04,0x78},//h
    {0x00,0x7D,0x00,0x00,0x00},//i
    {0x80,0x84,0x7D,0x00,0x00},//j
    {0x7F,0x10,0x28,0x44,0x00},//k
    {0x41,0x7F,0x40,0x00,0x00},//l
    {0x7C,0x04,0x18,0x04,0x78},//m
    {0x7C,0x08,0x04,0x7C,0x00},//n
    {0x38,0x44,0x44,0x38,0x00},//o
    {0xFC,0x24,0x24,0x18,0x00},//p
    {0x18,0x24,0x24,0xFC,0x00},//q
    {0x00,0x7C,0x08,0x04,0x00},//r
    {0x48,0x54,0x54,0x24,0x00},//s
    {0x04,0x7F,0x44,0x00,0x00},//t
    {0x3C,0x40,0x40,0x7C,0x00},//u
    {0x1C,0x20,0x40,0x20,0x1C},//v
    {0x3C,0x40,0x30,0x40,0x3C},//w
    {0x44,0x28,0x10,0x28,0x44},//x
    {0x1C,0xA0,0xA0,0x7C,0x00},//y
    {0x44,0x64,0x54,0x4C,0x44},//z
    {0x08,0x36,0x41,0x00,0x00},//{
    {0x00,0x7F,0x00,0x00,0x00},//|
    {0x41,0x36,0x08,0x00,0x00},//}
};

void delay_ms(unsigned int x){
    unsigned int i;
    for(i = 0; i < x; i++) __delay_ms(1);
}

void SPI_init(){
    SSPEN = 0;
    SMP = 0;
    CKE = 1;
    SSPCON1 = 0b00100000;
    SSPEN = 1;
    SSPIF = 0;
}

void SPI_write(unsigned char data){
    SSPBUF = data;
    while(!SSPIF);
    SSPIF = 0;
}

void start_frame(){
    SPI_write(0);
    SPI_write(0);
    SPI_write(0);
    SPI_write(0);
}

void end_frame(){
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
    SPI_write(255);
}

void LED_frame(unsigned char red, unsigned char green, unsigned char blue){
    SPI_write(255);
    SPI_write(blue);
    SPI_write(green);
    SPI_write(red);
}

void ColorLCD_writecommand(unsigned char c) {
    DC = 0;
    CS = 0;
    SPI_write(c);
    CS = 1;
}

void ColorLCD_writedata(unsigned char c){
    DC = 1;
    CS = 0;
    SPI_write(c);
    CS = 1;
} 

void ColorLCD_setxy(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1) {
    ColorLCD_writecommand(0x2A); 
    ColorLCD_writedata(x0 >> 8);
    ColorLCD_writedata(x0 & 0xFF); 
    ColorLCD_writedata(x1 >> 8);
    ColorLCD_writedata(x1 & 0xFF);  
    ColorLCD_writecommand(0x2B); 
    ColorLCD_writedata(y0 >> 8);
    ColorLCD_writedata(y0);   
    ColorLCD_writedata(y1 >> 8);
    ColorLCD_writedata(y1);    
    ColorLCD_writecommand(0x2C); 
}

void ColorLCD_fillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int color) {
    unsigned char hi = color >> 8, lo = color & 0xFF;
    ColorLCD_setxy(x, y, (x + w - 1), (y + h - 1));    
    DC = 1;
    CS = 0;
    for(y = h; y > 0; y--){
      for(x = w; x > 0; x--){
        SPI_write(hi);
        SPI_write(lo);
      }
    }
    CS = 1;
}

void ColorLCD_init(){
    delay_ms(150);
    CS = 1;
    RST = 1;
    delay_ms(50);
    RST = 0;
    delay_ms(50);
    RST = 1;
    delay_ms(50);
    ColorLCD_writecommand(0xEF);
    ColorLCD_writedata(0x03);
    ColorLCD_writedata(0x80);
    ColorLCD_writedata(0x02);

    ColorLCD_writecommand(0xCF);  
    ColorLCD_writedata(0x00); 
    ColorLCD_writedata(0XC1); 
    ColorLCD_writedata(0X30); 

    ColorLCD_writecommand(0xED);  
    ColorLCD_writedata(0x64); 
    ColorLCD_writedata(0x03); 
    ColorLCD_writedata(0X12); 
    ColorLCD_writedata(0X81); 

    ColorLCD_writecommand(0xE8);  
    ColorLCD_writedata(0x85); 
    ColorLCD_writedata(0x00); 
    ColorLCD_writedata(0x78); 

    ColorLCD_writecommand(0xCB);  
    ColorLCD_writedata(0x39); 
    ColorLCD_writedata(0x2C); 
    ColorLCD_writedata(0x00); 
    ColorLCD_writedata(0x34); 
    ColorLCD_writedata(0x02); 

    ColorLCD_writecommand(0xF7);  
    ColorLCD_writedata(0x20); 

    ColorLCD_writecommand(0xEA);  
    ColorLCD_writedata(0x00); 
    ColorLCD_writedata(0x00); 

    ColorLCD_writecommand(0xC0);    //Power control 
    ColorLCD_writedata(0x23);   //VRH[5:0] 

    ColorLCD_writecommand(0xC1);    //Power control 
    ColorLCD_writedata(0x10);   //SAP[2:0];BT[3:0] 

    ColorLCD_writecommand(0xC5);    //VCM control 
    ColorLCD_writedata(0x3e); //对比度调节
    ColorLCD_writedata(0x28); 

    ColorLCD_writecommand(0xC7);    //VCM control2 
    ColorLCD_writedata(0x86);  //--

    ColorLCD_writecommand(0x36);    // Memory Access Control 
    ColorLCD_writedata(0b00111000);

    ColorLCD_writecommand(0x3A);    
    ColorLCD_writedata(0x55); 

    ColorLCD_writecommand(0xB1);    
    ColorLCD_writedata(0x00);  
    ColorLCD_writedata(0x18); 

    ColorLCD_writecommand(0xB6);    // Display Function Control 
    ColorLCD_writedata(0x08); 
    ColorLCD_writedata(0x82);
    ColorLCD_writedata(0x27);  

    ColorLCD_writecommand(0xF2);    // 3Gamma Function Disable 
    ColorLCD_writedata(0x00); 

    ColorLCD_writecommand(0x26);    //Gamma curve selected 
    ColorLCD_writedata(0x01); 

    ColorLCD_writecommand(0xE0);    //Set Gamma 
    ColorLCD_writedata(0x0F); 
    ColorLCD_writedata(0x31); 
    ColorLCD_writedata(0x2B); 
    ColorLCD_writedata(0x0C); 
    ColorLCD_writedata(0x0E); 
    ColorLCD_writedata(0x08); 
    ColorLCD_writedata(0x4E); 
    ColorLCD_writedata(0xF1); 
    ColorLCD_writedata(0x37); 
    ColorLCD_writedata(0x07); 
    ColorLCD_writedata(0x10); 
    ColorLCD_writedata(0x03); 
    ColorLCD_writedata(0x0E); 
    ColorLCD_writedata(0x09); 
    ColorLCD_writedata(0x00); 

    ColorLCD_writecommand(0xE1);    //Set Gamma 
    ColorLCD_writedata(0x00); 
    ColorLCD_writedata(0x0E); 
    ColorLCD_writedata(0x14); 
    ColorLCD_writedata(0x03); 
    ColorLCD_writedata(0x11); 
    ColorLCD_writedata(0x07); 
    ColorLCD_writedata(0x31); 
    ColorLCD_writedata(0xC1); 
    ColorLCD_writedata(0x48); 
    ColorLCD_writedata(0x08); 
    ColorLCD_writedata(0x0F); 
    ColorLCD_writedata(0x0C); 
    ColorLCD_writedata(0x31); 
    ColorLCD_writedata(0x36); 
    ColorLCD_writedata(0x0F); 

    ColorLCD_writecommand(0x11);    //Exit Sleep 
    delay_ms(120); 		
    ColorLCD_writecommand(0x29);    //Display on 
    ColorLCD_fillRect(0, 0, 320, 250, 0xFFFF);
}

void ColorLCD_write_str(char *str, unsigned int x, unsigned int y, unsigned int color){
    unsigned int j, k, len = strlen(str);
    int i;
    char *t;
    ColorLCD_setxy(x, y, x + ((len * 6) - 1), (y + 7));
    DC = 1;
    CS = 0;
    for(i = 0; i < 8; i++){
        t = str;
        for(j = 0; j < len; j++, t++){
            for(k = 0; k < 5; k++){
                if((characters[*t - 32][k] >> i) & 1){
                    SPI_write(color >> 8);
                    SPI_write(color & 0xFF);
                }
                else{
                    SPI_write(0xFFFF);
                    SPI_write(0xFFFF);
                }
            }
            SPI_write(0xFFFF);
            SPI_write(0xFFFF);
        }
    }
    CS = 1;
}

void ColorLCD_write_int(int a, unsigned char precision, unsigned int x, unsigned int y, unsigned int color){
    char temp[10];
    unsigned char i = 1;
    if(a < 0){
        a *= (-1);
        temp[0] = '-';
    }
    else{
        temp[0] = '+';
    }
    if(precision >= 7) temp[i++] = (((a / 1000000) % 10) + 48);
    if(precision >= 6) temp[i++] = (((a / 100000) % 10) + 48);
    if(precision >= 5) temp[i++] = (((a / 10000) % 10) + 48);
    if(precision >= 4) temp[i++] = (((a / 1000) % 10) + 48);
    if(precision >= 3) temp[i++] = (((a / 100) % 10) + 48);
    if(precision >= 2) temp[i++] = (((a / 10) % 10) + 48);
    if(precision >= 1) temp[i++] = ((a % 10) + 48);
    temp[i] = '\0';
    ColorLCD_write_str(temp, x, y, color);
}

void ColorLCD_write_float(double a, unsigned char left, unsigned char right, unsigned int x, unsigned int y, unsigned int color){
    char temp[20];
    unsigned char i = 1, j;
    long int tens = 10;
    if(a < 0){
        a *= (-1);
        temp[0] = '-';
    }
    else{
        temp[0] = '+';
    }
    if(left >= 6) temp[i++] = (((int)(a / 100000) % 10) + 48);
    if(left >= 5) temp[i++] = (((int)(a / 10000) % 10) + 48);
    if(left >= 4) temp[i++] = (((int)(a / 1000) % 10) + 48);
    if(left >= 3) temp[i++] = (((int)(a / 100) % 10) + 48);
    if(left >= 2) temp[i++] = (((int)(a / 10) % 10) + 48);
    if(left >= 1) temp[i++] = (((int)a % 10) + 48);
    temp[i++] = '.';
    for(j = 0; j < right; j++){
        temp[i++] = (((long int)(a * tens) % 10) + 48);
        tens *= 10;
    }
    temp[i] = '\0';
    ColorLCD_write_str(temp, x, y, color);
}
 
void init(){
    TRISA = 63;
    TRISB = 0;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
}

void main(){
    unsigned char i, j;
    unsigned int r, g, b;
    srand(TMR0);
    init();
    SPI_init();
    ColorLCD_init();
    while(1){
        for(i = 0; i < 120 ;i += 1){
            r = rand() & 0b11111;
            g = rand() & 0b111111;
            b = rand() & 0b11111;
            ColorLCD_fillRect(i, i, (320 - 2 * i), (240 - 2 * i), (r << 11 | g << 5 | b));
        }
        delay_ms(1000);
    }
}