#define _XTAL_FREQ 20000000
#include <xc.h>
#include <pic16f877a.h>

#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config BOREN = OFF
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

const int strip_length = 48;

int LED_flash_counter, fabulous_counter, morph_counter;

void UART_init(){
    TXSTA = 36;
    SPBRG = 129;
    RCSTA = 144;
    TXIE = 0;
    RCIE = 0;
}

void send_byte(unsigned char byte){
    while(!TXIF);
    TXREG = byte;
}

unsigned char receive_byte(){
    if(OERR){
        CREN = 0;
        CREN = 1;
    }
    while(!RCIF);
    return RCREG;
}

void SPI_init(){
    SSPEN = 0;
    SMP = 0;
    CKE = 1;
    SSPCON = 0b00100000;
    SSPEN = 1;
    SSPIF = 0;
}

void SPI_write(unsigned char data){
    SSPBUF = data;
    while (!SSPIF);
    SSPIF = 0;
}

void init(){
    TRISB = 0b11000000;
    TRISC = 128;
    PORTB = 0;
    PORTC = 0;
    //UART_init();
    SPI_init();
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

void fabulous(){
    int j, k, r, g, b;
    const float ka = 255 / 38;//16
    const float kb = 255 / 39;//15
    start_frame();
    for(j = fabulous_counter, k = 0; j < strip_length; j++, k++){
        if(j < 39){
            r = 255;
            g = (int)(float)(ka * j);
            b = 0;
        }
        else if(j < 79){
            r = (int)(float)(kb * (39 - (j - 39)));
            g = 255;
            b = 0;
        }
        else if(j < 118){
            r = 0;
            g = 255;
            b = (int)(float)(ka * (j - 79));
        }
        else if(j < 157){
            r = 0;
            g = (int)(float)(ka * (38 - (j - 118)));
            b = 255;
        }
        else if(j < 197){
            r = (int)(float)(kb * (j - 157));
            g = 0;
            b = 255;
        }
        else{
            r = 255;
            g = 0;
            b = (int)(float)(ka * (38 - (j - 197)));
        }
        LED_frame(r, g, b);
    }
    for(j = 0; j < fabulous_counter; j++, k++){
        if(j < 39){
            r = 255;
            g = (int)(float)(ka * j);
            b = 0;
        }
        else if(j < 79){
            r = (int)(float)(kb * (39 - (j - 39)));
            g = 255;
            b = 0;
        }
        else if(j < 118){
            r = 0;
            g = 255;
            b = (int)(float)(ka * (j - 79));
        }
        else if(j < 157){
            r = 0;
            g = (int)(float)(ka * (38 - (j - 118)));
            b = 255;
        }
        else if(j < 197){
            r = (int)(float)(kb * (j - 157));
            g = 0;
            b = 255;
        }
        else{
            r = 255;
            g = 0;
            b = (int)(float)(ka * (38 - (j - 197)));
        }
        LED_frame(r, g, b);
    }
    end_frame();
    fabulous_counter += 5;
    if(fabulous_counter == strip_length){
        fabulous_counter = 0;
    }
}

void LED_flash(){
    int i;
    start_frame();
    for(i = 0; i < strip_length; i++){
        if(LED_flash_counter == 0){
            if(i % 3 == 0){
                LED_frame(0, 255, 0);
            }
            else if(i % 3 == 1){
                LED_frame(190, 190, 190);
            }
            else{
                LED_frame(255, 128, 0);
            }
        }
        else if(LED_flash_counter == 1){
            if(i % 3 == 0){
                LED_frame(255, 128, 0);
            }
            else if(i % 3 == 1){
                LED_frame(0, 255, 0);
            }
            else{
                LED_frame(190, 190, 190);
            }
        }
        else{
            if(i % 3 == 0){
                LED_frame(190, 190, 190);
            }
            else if(i % 3 == 1){
                LED_frame(255, 128, 0);
            }
            else{
                LED_frame(0, 255, 0);
            }
        }
    }
    end_frame();
    __delay_ms(100);
    if(++LED_flash_counter == 3){
        LED_flash_counter = 0;
    }
}

void morph(){
    int r, g, b, i;
    if(morph_counter < 256){
        r = 255;
        g = morph_counter;
        b = 0;
    }
    else if(morph_counter < 512){
        r = 255 - (morph_counter - 256);
        g = 255;
        b = 0;
    }
    else if(morph_counter < 768){
        r = 0;
        g = 255;
        b = morph_counter - 512;
    }
    else if(morph_counter < 1024){
        r = 0;
        g = 255 - (morph_counter - 768);
        b = 255;
    }
    else if(morph_counter < 1280){
        r = morph_counter - 1024;
        g = 0;
        b = 255;
    }
    else{
        r = 255;
        g = 0;
        b = 255 - (morph_counter - 1280);
    }
    start_frame();
    for(i = 0; i < strip_length; i++){
        LED_frame(r, g, b);
    }
    end_frame();
    morph_counter += 3;
    if(morph_counter == 1536){
        morph_counter = 0;
    }
}

void blank(){
    int i;
    start_frame();
    for(i = 0; i < strip_length; i++){
        LED_frame(0, 0, 0);
    }
    end_frame();
}

void main(){
    init();
    __delay_ms(50);
    blank();
    fabulous_counter = 0;
    morph_counter = 0;
    LED_flash_counter = 0;
    __delay_ms(500);
    
    int r, g, b, i, j, k;
    while(1){
        for(k = 0; k < 96; k++){
            start_frame();
            for(j = 0; j < 3; j++){
                for(i = 0, morph_counter = k * 16; i < 16; i++, morph_counter += 16){
                    //morph_counter = morph_counter % 1536;
                    if(morph_counter == 1536){
                        morph_counter = 0;
                    }
                    if(morph_counter < 256){
                        r = 255;
                        g = morph_counter;
                        b = 0;
                    }
                    else if(morph_counter < 512){
                        r = 255 - (morph_counter - 256);
                        g = 255;
                        b = 0;
                    }
                    else if(morph_counter < 768){
                        r = 0;
                        g = 255;
                        b = morph_counter - 512;
                    }
                    else if(morph_counter < 1024){
                        r = 0;
                        g = 255 - (morph_counter - 768);
                        b = 255;
                    }
                    else if(morph_counter < 1280){
                        r = morph_counter - 1024;
                        g = 0;
                        b = 255;
                    }
                    else{
                        r = 255;
                        g = 0;
                        b = 255 - (morph_counter - 1280);
                    }

                    LED_frame(r, g, b);
                }
            }
            end_frame();
            __delay_ms(15);
        }
    }
    /*while(1){
        morph();
    }*/
}