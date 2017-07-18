#define _XTAL_FREQ 48000000
#include <xc.h>
#include <pic18f4550.h>

#pragma config PLLDIV = 12
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


#define c1_red PORTDbits.RD7
#define c1_blue PORTDbits.RD6
#define c1_green PORTDbits.RD5 

#define c2_red PORTDbits.RD4
#define c2_blue PORTCbits.RC7
#define c2_green PORTCbits.RC6

#define c3_red PORTDbits.RD3
#define c3_blue PORTDbits.RD2
#define c3_green PORTDbits.RD1

#define c4_red PORTCbits.RC0
#define c4_blue PORTCbits.RC1
#define c4_green PORTCbits.RC2

#define c5_red PORTAbits.RA3
#define c5_blue PORTAbits.RA4
#define c5_green PORTAbits.RA5

#define c6_red PORTEbits.RE0
#define c6_blue PORTEbits.RE1
#define c6_green PORTEbits.RE2

#define row1 PORTBbits.RB0
#define row2 PORTBbits.RB1
#define row3 PORTBbits.RB2
#define row4 PORTBbits.RB3
#define row5 PORTBbits.RB4
#define row6 PORTBbits.RB5

unsigned char buffer[6][6][3], c;

void delay_ms(int x){
    int i;
    for(i = 0; i < x; i++){
        __delay_ms(1);
    }
}

void delay_us(int x){
    int i;
    for(i = 0; i < x; i++){
        __delay_us(1);
    }
}

void timer_init(){
    T0CS = 0;       
    T0SE = 0;          
    PSA = 0;          
    T0PS2 = 0;
    T0PS1 = 0;
    T0PS0 = 1;    
    IPEN = 0;         
    GIE = 1;          
    TMR0 = 0;
    TMR0IE = 1;
    TMR0ON = 1;
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
    while (!SSPIF);
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
}

void LED_frame(unsigned char brightness, unsigned char red, unsigned char green, unsigned char blue){
    SPI_write(224 + (brightness & 0x1F));
    SPI_write(blue & 0xFF);
    SPI_write(green & 0xFF);
    SPI_write(red & 0xFF);
}

int get_adc(unsigned char port){
    ADCON0 = port * 8 + 1;
    GO_DONE = 1;
    while(GO_DONE);
    return ((ADRESH  * 256) + ADRESL);
}

void reset(){
    row1 = 0;
    row2 = 0;
    row3 = 0;
    row4 = 0;
    row5 = 0;
    row6 = 0;
    c1_red = 1;
    c1_green = 1;
    c1_blue = 1;
    c2_red = 1;
    c2_green = 1;
    c2_blue = 1;
    c3_red = 1;
    c3_green = 1;
    c3_blue = 1;
    c4_red = 1;
    c4_green = 1;
    c4_blue = 1;
    c5_red = 1;
    c5_green = 1;
    c5_blue = 1;
    c6_red = 1;
    c6_green = 1;
    c6_blue = 1;
}

void interrupt ISR(){
    if(TMR0IF){
        TMR0IF = 0;
        if(c == 0){
            row6 = 0;
            c1_red = buffer[0][0][0];
            c1_green = buffer[0][0][1];
            c1_blue = buffer[0][0][2];
            c2_red = buffer[0][1][0];
            c2_green = buffer[0][1][1];
            c2_blue = buffer[0][1][2];
            c3_red = buffer[0][2][0];
            c3_green = buffer[0][2][1];
            c3_blue = buffer[0][2][2];
            c4_red = buffer[0][3][0];
            c4_green = buffer[0][3][1];
            c4_blue = buffer[0][3][2];
            c5_red = buffer[0][4][0];
            c5_green = buffer[0][4][1];
            c5_blue = buffer[0][4][2];
            c6_red = buffer[0][5][0];
            c6_green = buffer[0][5][1];
            c6_blue = buffer[0][5][2];
            row1 = 1;
            c++;
        }
        else if(c == 1){
            row1 = 0;
            c1_red = buffer[1][0][0];
            c1_green = buffer[1][0][1];
            c1_blue = buffer[1][0][2];
            c2_red = buffer[1][1][0];
            c2_green = buffer[1][1][1];
            c2_blue = buffer[1][1][2];
            c3_red = buffer[1][2][0];
            c3_green = buffer[1][2][1];
            c3_blue = buffer[1][2][2];
            c4_red = buffer[1][3][0];
            c4_green = buffer[1][3][1];
            c4_blue = buffer[1][3][2];
            c5_red = buffer[1][4][0];
            c5_green = buffer[1][4][1];
            c5_blue = buffer[1][4][2];
            c6_red = buffer[1][5][0];
            c6_green = buffer[1][5][1];
            c6_blue = buffer[1][5][2];
            row2 = 1;
            c++;
        }
        else if(c == 2){
            row2 = 0;
            c1_red = buffer[2][0][0];
            c1_green = buffer[2][0][1];
            c1_blue = buffer[2][0][2];
            c2_red = buffer[2][1][0];
            c2_green = buffer[2][1][1];
            c2_blue = buffer[2][1][2];
            c3_red = buffer[2][2][0];
            c3_green = buffer[2][2][1];
            c3_blue = buffer[2][2][2];
            c4_red = buffer[2][3][0];
            c4_green = buffer[2][3][1];
            c4_blue = buffer[2][3][2];
            c5_red = buffer[2][4][0];
            c5_green = buffer[2][4][1];
            c5_blue = buffer[2][4][2];
            c6_red = buffer[2][5][0];
            c6_green = buffer[2][5][1];
            c6_blue = buffer[2][5][2];
            row3 = 1;
            c++;
        }
        else if(c == 3){
            row3 = 0;
            c1_red = buffer[3][0][0];
            c1_green = buffer[3][0][1];
            c1_blue = buffer[3][0][2];
            c2_red = buffer[3][1][0];
            c2_green = buffer[3][1][1];
            c2_blue = buffer[3][1][2];
            c3_red = buffer[3][2][0];
            c3_green = buffer[3][2][1];
            c3_blue = buffer[3][2][2];
            c4_red = buffer[3][3][0];
            c4_green = buffer[3][3][1];
            c4_blue = buffer[3][3][2];
            c5_red = buffer[3][4][0];
            c5_green = buffer[3][4][1];
            c5_blue = buffer[3][4][2];
            c6_red = buffer[3][5][0];
            c6_green = buffer[3][5][1];
            c6_blue = buffer[3][5][2];
            row4 = 1;
            c++;
        }
        else if(c == 4){
            row4 = 0;
            c1_red = buffer[4][0][0];
            c1_green = buffer[4][0][1];
            c1_blue = buffer[4][0][2];
            c2_red = buffer[4][1][0];
            c2_green = buffer[4][1][1];
            c2_blue = buffer[4][1][2];
            c3_red = buffer[4][2][0];
            c3_green = buffer[4][2][1];
            c3_blue = buffer[4][2][2];
            c4_red = buffer[4][3][0];
            c4_green = buffer[4][3][1];
            c4_blue = buffer[4][3][2];
            c5_red = buffer[4][4][0];
            c5_green = buffer[4][4][1];
            c5_blue = buffer[4][4][2];
            c6_red = buffer[4][5][0];
            c6_green = buffer[4][5][1];
            c6_blue = buffer[4][5][2];
            row5 = 1;
            c++;
        }
        else if(c == 5){
            row5 = 0;
            c1_red = buffer[5][0][0];
            c1_green = buffer[5][0][1];
            c1_blue = buffer[5][0][2];
            c2_red = buffer[5][1][0];
            c2_green = buffer[5][1][1];
            c2_blue = buffer[5][1][2];
            c3_red = buffer[5][2][0];
            c3_green = buffer[5][2][1];
            c3_blue = buffer[5][2][2];
            c4_red = buffer[5][3][0];
            c4_green = buffer[5][3][1];
            c4_blue = buffer[5][3][2];
            c5_red = buffer[5][4][0];
            c5_green = buffer[5][4][1];
            c5_blue = buffer[5][4][2];
            c6_red = buffer[5][5][0];
            c6_green = buffer[5][5][1];
            c6_blue = buffer[5][5][2];
            row6 = 1;
            c = 0;
        }
    }
}    

void main(){
    int i, j;
    TRISA = 0;
    TRISB = 0;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    USBEN = 0;
    UTRDIS = 1;
    //SPI_init();
    ADON = 0;
    ADCON1 = 15;
    //ADCON2 = 184;
    reset();
    c = 0;
    for(i = 0; i < 6; i++){
        for(j = 0; j < 6; j++){
            buffer[i][j][0] = 1;
            buffer[i][j][1] = 1;
            buffer[i][j][2] = 1;
        }
    }
    
    timer_init();
    while(1){
        for(i = 0; i < 6; i++){
            buffer[0][i][0] = 1;
            buffer[0][i][1] = 1;
            buffer[0][i][2] = 0;
            delay_ms(100);
        }
        for(i = 1; i < 6; i++){
            buffer[i][5][0] = 1;
            buffer[i][5][1] = 1;
            buffer[i][5][2] = 0;
            delay_ms(100);
        }
        for(i = 4; i >= 0; i--){
            buffer[5][i][0] = 1;
            buffer[5][i][1] = 1;
            buffer[5][i][2] = 0;
            delay_ms(100);
        }
        for(i = 4; i >= 1; i--){
            buffer[i][0][0] = 1;
            buffer[i][0][1] = 1;
            buffer[i][0][2] = 0;
            delay_ms(100);
        }
        
        for(i = 1; i < 5; i++){
            buffer[1][i][0] = 1;
            buffer[1][i][1] = 1;
            buffer[1][i][2] = 0;
            delay_ms(100);
        }
        for(i = 2; i < 5; i++){
            buffer[i][4][0] = 1;
            buffer[i][4][1] = 1;
            buffer[i][4][2] = 0;
            delay_ms(100);
        }
        for(i = 3; i >= 1; i--){
            buffer[4][i][0] = 1;
            buffer[4][i][1] = 1;
            buffer[4][i][2] = 0;
            delay_ms(100);
        }
        for(i = 3; i >= 2; i--){
            buffer[i][1][0] = 1;
            buffer[i][1][1] = 1;
            buffer[i][1][2] = 0;
            delay_ms(100);
        }
        buffer[2][2][0] = 1;
        buffer[2][2][1] = 1;
        buffer[2][2][2] = 0;      
        delay_ms(100);
        buffer[2][3][0] = 1;
        buffer[2][3][1] = 1;
        buffer[2][3][2] = 0;     
        delay_ms(100);
        buffer[3][3][0] = 1;
        buffer[3][3][1] = 1;
        buffer[3][3][2] = 0;     
        delay_ms(100);
        buffer[3][2][0] = 1;
        buffer[3][2][1] = 1;
        buffer[3][2][2] = 0; 
        delay_ms(1000);
        for(i = 0; i < 6; i++){
            for(j = 0; j < 6; j++){
                buffer[j][i][0] = 1;
                buffer[j][i][1] = 1;
                buffer[j][i][2] = 1;
            }
        }
    }
}