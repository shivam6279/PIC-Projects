#include "SPI.h"
#include "LED.h"
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
}

void start_frame4(){
    SPI4_write(0);
    SPI4_write(0);
    SPI4_write(0);
    SPI4_write(0);
}

void end_frame4(){
    SPI4_write(255);
    SPI4_write(255);
    SPI4_write(255);
    SPI4_write(255);
    SPI4_write(255);
    SPI4_write(255);
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

void LED_frame4(unsigned char red, unsigned char green, unsigned char blue){
    float r = (float)red * brightness;
    float g = (float)green * brightness;
    float b = (float)blue * brightness;
    if(r > 255.0) r = 255.0;
    if(g > 255.0) g = 255.0;
    if(b > 255.0) b = 255.0;
    SPI4_write(255);
    SPI4_write((unsigned char)b);
    SPI4_write((unsigned char)g);
    SPI4_write((unsigned char)r);
}

void start_frame_double(){
    SPI_write_double(0);
    SPI_write_double(0);
    SPI_write_double(0);
    SPI_write_double(0);
}

void end_frame_double(){
    SPI_write_double(255);
    SPI_write_double(255);
    SPI_write_double(255);
    SPI_write_double(255);
    SPI_write_double(255);
    SPI_write_double(255);
}

void LED_frame_double(struct led a, struct led b) {
    static float ra, ga, ba, rb, gb, bb;
    ra = (float)a.red * brightness;
    ga = (float)a.green * brightness;
    ba = (float)a.blue * brightness;
    
    rb = (float)b.red * brightness;
    gb = (float)b.green * brightness;
    bb = (float)b.blue * brightness;
    
    if(ra > 255.0) ra = 255.0;
    if(ga > 255.0) ga = 255.0;
    if(ba > 255.0) ba = 255.0;
    
    if(rb > 255.0) rb = 255.0;
    if(gb > 255.0) gb = 255.0;
    if(bb > 255.0) bb = 255.0;
    
    SPI_write_double(255);
    SPI_write_double((unsigned char)ba, (unsigned char)bb);
    SPI_write_double((unsigned char)ga, (unsigned char)gb);
    SPI_write_double((unsigned char)ra, (unsigned char)rb);
}

void writeLEDs(struct led buffer[2][LED_LENGTH + RADIUS_OFFSET]) {
    int i;
    
//    start_frame_double();
//    for(i = RADIUS_OFFSET; i < (LED_LENGTH + RADIUS_OFFSET); i++){
//        LED_frame_double(buffer[0][i], buffer[1][i]);
//    }
//    end_frame_double();
    
    start_frame();
    for(i = RADIUS_OFFSET; i < (LED_LENGTH + RADIUS_OFFSET); i++){
        LED_frame((unsigned char)buffer[0][i].red, (unsigned char)buffer[0][i].green, (unsigned char)buffer[0][i].blue);
    }
    end_frame();
}

/*
void writeLEDs_hue(struct led buffer[2][LED_LENGTH + RADIUS_OFFSET], float hue) {
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
*/