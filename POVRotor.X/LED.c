#include "SPI.h"
#include "LED.h"

const float brightness = 0.3;

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