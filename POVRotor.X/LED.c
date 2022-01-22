#include "SPI.h"
#include "LED.h"
#include "draw.h"
#include <math.h>
#include <xc.h>
#include "pic32.h"
#include <sys/attribs.h>  

#define LED_BRIGTHNESS 0.3
static unsigned char br_byte = 0b11100000 | (unsigned char)((float)LED_BRIGTHNESS*31);

static const int led_length_2 = LED_LENGTH / 2;

struct led color_white = {255, 255, 255};
struct led color_black = {0, 0, 0};
struct led color_red = {255, 0, 0};
struct led color_green = {0, 255, 0};
struct led color_blue = {0, 0, 255};
struct led color_cyan = {0, 255, 255};
struct led color_magenta = {255, 0, 255};
struct led color_yellow = {255, 255, 0};

struct led buffer[LED_LENGTH];

static volatile char LED_tx_buffer[BUFFER_LENGTH];
static volatile unsigned int LED_tx_buffer_index = 0;

void __ISR_AT_VECTOR(_SPI4_TX_VECTOR, IPL6AUTO) APA102_TX(void) {
    static unsigned char i;
    IFS5bits.SPI4TXIF = 0;
    if(LED_tx_buffer_index) {
        for(i = 0; i < 16; i++) {
            SPI4BUF = LED_tx_buffer[LED_tx_buffer_index++];
            if(LED_tx_buffer_index == BUFFER_LENGTH) {
                LED_tx_buffer_index = 0;
                LED_TX_INTERRUPT = 0;
                break;
            }
        }
    }
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
}

void LED_frame(unsigned char red, unsigned char green, unsigned char blue){
    SPI_write(br_byte);
    SPI_write(blue);
    SPI_write(green);
    SPI_write(red);
}

void scaleBrightness(struct led *buffer, float min_scale) {
    int i;
    float r, scale;
    float min_scale_1 = 1.0 - min_scale;
    
    for(i = 0; i < led_length_2; i++) {
        r = i;
        scale = r/(float)led_length_2 * min_scale_1 + min_scale;
        buffer[LED_LENGTH / 2 - 1 - i].red   = (float)buffer[LED_LENGTH / 2 - 1 - i].red   * scale;
        buffer[LED_LENGTH / 2 - 1 - i].green = (float)buffer[LED_LENGTH / 2 - 1 - i].green * scale;
        buffer[LED_LENGTH / 2 - 1 - i].blue  = (float)buffer[LED_LENGTH / 2 - 1 - i].blue  * scale;
        
        buffer[LED_LENGTH / 2 + i].red   = (float)buffer[LED_LENGTH / 2 + i].red   * scale;
        buffer[LED_LENGTH / 2 + i].green = (float)buffer[LED_LENGTH / 2 + i].green * scale;
        buffer[LED_LENGTH / 2 + i].blue  = (float)buffer[LED_LENGTH / 2 + i].blue  * scale;
    }
    
//    for(i = 0; i < LED_LENGTH/2; i++) {
//        r = (double)i + 0.75;
//        scale = r * 2.0 / (float)LED_LENGTH * (1.0 - min_scale) + min_scale;
//        buffer[LED_LENGTH / 2 - 1 - i].red   = (float)buffer[LED_LENGTH / 2 - 1 - i].red   * scale;
//        buffer[LED_LENGTH / 2 - 1 - i].green = (float)buffer[LED_LENGTH / 2 - 1 - i].green * scale;
//        buffer[LED_LENGTH / 2 - 1 - i].blue  = (float)buffer[LED_LENGTH / 2 - 1 - i].blue  * scale;
//    }
//    
//    for(i = 0; i < LED_LENGTH/2; i++) {
//        r = (double)i + 0.25;
//        scale = r * 2.0 / (float)LED_LENGTH * (1.0 - min_scale) + min_scale;
//        buffer[LED_LENGTH / 2 + i].red   = (float)buffer[LED_LENGTH / 2 + i].red   * scale;
//        buffer[LED_LENGTH / 2 + i].green = (float)buffer[LED_LENGTH / 2 + i].green * scale;
//        buffer[LED_LENGTH / 2 + i].blue  = (float)buffer[LED_LENGTH / 2 + i].blue  * scale;
//    }
}

void writeLEDs(struct led *buffer) {
    unsigned int i, j;
    
    start_frame();
    for(i = 0; i < LED_LENGTH; i++){        
        LED_frame(buffer[i].red, buffer[i].green, buffer[i].blue);
    }
    end_frame();
}

inline void writeLEDs_ISR(struct led *buffer) {
    unsigned int i, j;  
    
    LED_TX_INTERRUPT = 0;    
    
    for(i = 0; i < 4; i++) {
        LED_tx_buffer[i] = 0;
    }    
    for(j = 0; j < LED_LENGTH; j++) {        
        LED_tx_buffer[i++] = br_byte;
        LED_tx_buffer[i++] = buffer[j].blue;
        LED_tx_buffer[i++] = buffer[j].green;
        LED_tx_buffer[i++] = buffer[j].red;
    }
    for(j = 0; j < 8; j++) {
        LED_tx_buffer[i++] = 255;
    }
    
    LED_tx_buffer_index = 0;
    IFS5bits.SPI4TXIF = 0;
    
//    while(!SPI4STATbits.SPITBF) {
//        SPI4BUF = LED_tx_buffer[LED_tx_buffer_index++];    
//    }
    for(i = 0; i < 16; i++) {
        SPI4BUF = LED_tx_buffer[LED_tx_buffer_index++];  
    }
    LED_TX_INTERRUPT = 1;
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

void led_test_loop() {
    int j;
    while(1) {
        for(j = 0; j < LED_LENGTH; j++){
            buffer[j] = color_red;
        }
        for(j = 0; j < 250; j++){
            writeLEDs(buffer);
            delay_ms(4);
        }
        
        for(j = 0; j < LED_LENGTH; j++){
            buffer[j] = color_blue;
        }
        for(j = 0; j < 250; j++){
            writeLEDs(buffer);
            delay_ms(4);
        }
        
        for(j = 0; j < LED_LENGTH; j++){
            buffer[j] = color_green;
        }
        for(j = 0; j < 250; j++){
            writeLEDs(buffer);
            delay_ms(4);
        }
    }   
}