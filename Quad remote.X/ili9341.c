#include "ili9341.h"
#include "SPI.h"

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