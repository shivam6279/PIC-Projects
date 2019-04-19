#include "ili9341.h"
#include "SPI.h"

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

void ColorLCD_writecommand(unsigned char c){
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

void ColorLCD_setxy(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1){
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

void ColorLCD_init(){
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
}