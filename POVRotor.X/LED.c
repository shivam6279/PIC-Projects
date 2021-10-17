#include "SPI.h"
#include "LED.h"
#include "draw.h"
#include <math.h>

const float brightness = 0.3;

struct led color_white = {255, 255, 255};
struct led color_black = {0, 0, 0};
struct led color_red = {255, 0, 0};
struct led color_green = {0, 255, 0};
struct led color_blue = {0, 0, 255};
struct led color_cyan = {0, 255, 255};
struct led color_magenta = {255, 0, 255};
struct led color_yellow = {255, 255, 0};

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
}

void LED_frame(unsigned char red, unsigned char green, unsigned char blue){
    float r = (float)red * brightness;
    float g = (float)green * brightness;
    float b = (float)blue * brightness;
    if(r > 255.0) r = 255.0;
    if(g > 255.0) g = 255.0;
    if(b > 255.0) b = 255.0;
    SPI_write(255);
    SPI_write((unsigned char)b);
    SPI_write((unsigned char)g);
    SPI_write((unsigned char)r);
}

void writeLEDs(struct led *buffer) {
    int i;
    
    start_frame();
    for(i = 0; i < LED_LENGTH; i++){
        LED_frame(buffer[i].red, buffer[i].green, buffer[i].blue);
    }
    end_frame();
}

void writeLEDs_hue(struct led *buffer, double hue) {
    int i;
    
    limit_angle(&hue);
    float r, g, b;
    float cosA = cos(hue*3.14159265f/180); 
    float sinA = sin(hue*3.14159265f/180);
    float matrix[3][3] = {{cosA + (1.0f - cosA) / 3.0f, 1.0f/3.0f * (1.0f - cosA) - sqrtf(1.0f/3.0f) * sinA, 1.0f/3.0f * (1.0f - cosA) + sqrtf(1.0f/3.0f) * sinA},
        {1.0f/3.0f * (1.0f - cosA) + sqrtf(1.0f/3.0f) * sinA, cosA + 1.0f/3.0f*(1.0f - cosA), 1.0f/3.0f * (1.0f - cosA) - sqrtf(1.0f/3.0f) * sinA},
        {1.0f/3.0f * (1.0f - cosA) - sqrtf(1.0f/3.0f) * sinA, 1.0f/3.0f * (1.0f - cosA) + sqrtf(1.0f/3.0f) * sinA, cosA + 1.0f/3.0f * (1.0f - cosA)}};
    
    start_frame();
    for(i = 0; i < LED_LENGTH; i++){
        r = buffer[i].red*matrix[0][0] + buffer[i].green*matrix[0][1] + buffer[i].blue*matrix[0][2];
        g = buffer[i].red*matrix[1][0] + buffer[i].green*matrix[1][1] + buffer[i].blue*matrix[1][2];
        b = buffer[i].red*matrix[2][0] + buffer[i].green*matrix[2][1] + buffer[i].blue*matrix[2][2];
        
        if(r < 0) 
            r = 0;
        else if (r > 255)
            r = 255;
        if(g < 0) 
            g = 0;
        else if (g > 255)
            g = 255;
        if(b < 0) 
            b = 0;
        else if (b > 255)
            b = 255;
        
        LED_frame((unsigned char)r, (unsigned char)g, (unsigned char)b);
    }
    end_frame();
}