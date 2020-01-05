#include "draw.h"
#include "ili9341.h"
#include <string.h>

void DrawPixel(unsigned int x, unsigned int y, unsigned int color){
    ColorLCD_setxy(x, y, (x + 1), (y + 1));
    DC = 1;
    CS = 0;
    SPI_write(color >> 8);
    SPI_write(color & 0xFF);
    CS = 1;
}

void WriteStr(char *str, unsigned int x, unsigned int y, unsigned int color){
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
                    SPI_write(0xFF);
                    SPI_write(0xFF);
                }
            }
            SPI_write(0xFF);
            SPI_write(0xFF);
        }
    }
    CS = 1;
}

void WriteInt(int a, unsigned char precision, unsigned int x, unsigned int y, unsigned int color){
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
    WriteStr(temp, x, y, color);
}

void WriteFloat(double a, unsigned char left, unsigned char right, unsigned int x, unsigned int y, unsigned int color){
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
    WriteStr(temp, x, y, color);
}

void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int color){
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

float mag(float a){
    if(a < 0) return -a;
    else return a;
}

float sign(float a){
    if(a < 0) return -1.0;
    else return 1.0;
}

void DrawLine(int x1, int y1, int x2, int y2, unsigned int color){
    float dx = x2 - x1;
    float dy = y2 - y1;
    float e = 0.0, de;
    int x = x1, y = y1;
    if(y1 == y2){
        for(; x < x2; x++){
            DrawPixel(x, y, color);
        }
    }
    else if(x1 == x2){
        for(; y < y2; y++){
            DrawPixel(x, y, color);
        }
    }
    else{
        de = mag(dy/dx);
        for(; x < x2; x++){
            DrawPixel(x, y, color);
            e += de;
            while(e >= 0.5){
                y += sign(dy);
                e -= 1.0;
            }
        }
    }
    
}

void DrawCircle(int x0, int y0, int radius, unsigned int color){
	int x = 0, y = radius;
	int dp = 1 - radius;
	do{
		if (dp < 0) dp = dp + 2 * (++x) + 3;
		else dp = dp + 2 * (++x) - 2 * (--y) + 5;

		DrawPixel(x0 + x, y0 + y, color);     //For the 8 octants
		DrawPixel(x0 - x, y0 + y, color);
		DrawPixel(x0 + x, y0 - y, color);
		DrawPixel(x0 - x, y0 - y, color);
		DrawPixel(x0 + y, y0 + x, color);
		DrawPixel(x0 - y, y0 + x, color);
		DrawPixel(x0 + y, y0 - x, color);
		DrawPixel(x0 - y, y0 - x, color);

	}while(x < y);
    DrawPixel(x0 + radius, y0, color);
    DrawPixel(x0 - radius, y0, color);
    DrawPixel(x0, y0 + radius, color);
    DrawPixel(x0, y0 - radius, color);
}