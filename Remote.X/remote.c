#define _XTAL_FREQ 48000000
#include <xc.h>
#include <pic18f4550.h>
#include <math.h>
#include "USART.h"
#include "I2C.h"
#include "OLED.h"
#include "MPU6050.h"

#define button_A PORTDbits.RD6
#define button_B PORTDbits.RD5
#define button_X PORTDbits.RD7
#define button_Y PORTDbits.RD4

#define button_L1 PORTCbits.RC2
#define button_L2 PORTCbits.RC1
#define button_R1 PORTBbits.RB2
#define button_R2 PORTBbits.RB3

#define button_L3 PORTCbits.RC5
#define button_R3 PORTCbits.RC4

#define button_up PORTDbits.RD0
#define button_down PORTDbits.RD2
#define button_left PORTDbits.RD1
#define button_right PORTDbits.RD3

#define rumble_left PORTEbits.RE0
#define rumble_right PORTAbits.RA5

#define mode_quad 0
#define mode_dpad 1
#define mode_receiver 2

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

unsigned char c, mode_quad_counter;
signed char cursor;
int throttle, direction, x1, x2, y1, y2, receive, buttons_send, x1_send, y1_send;

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

void timer_init(unsigned char a){
    T0CS = 0;           //a = 7: 1:256
    T0SE = 0;           //a = 6: 1:128
    PSA = 0;            //a = 5: 1:64
    T0PS2 = (a / 4) % 2;//a = 4: 1:32
    T0PS1 = (a / 2) % 2;//a = 3: 1:16
    T0PS0 = a % 2;      //a = 2: 1:8
    IPEN = 0;           //a = 1: 1:4
    GIE = 1;            //a = 0: 1:2
    TMR0 = 0;
    TMR0IE = 1;
    TMR0ON = 1;
}

int get_adc(unsigned char port){
    ADCON0 = port * 8 + 1;
    GO_DONE = 1;
    while(GO_DONE);
    return ((ADRESH  * 256) + ADRESL);
}

void get_adc_values(unsigned char a){
    int i;
    x1 = 0;
    y1 = 0;
    for(i = 0; i < 10; i++){
        x1 += get_adc(3);
        __delay_us(50);
    }
    x1 /= 10;
    for(i = 0; i < 10; i++){
        y1 += get_adc(1);
        __delay_us(50);
    }
    y1 /= 10;
    if(a){
        x2 = 0;
        for(i = 0; i < 10; i++){
            x2 += get_adc(0);
            __delay_us(50);
        }
        x2 /= 10;
        y2 = 0;
        for(i = 0; i < 10; i++){
            y2 += get_adc(2);
            __delay_us(50);
        }
        y2 /= 10;
        x2 -= 512;
        y2 -= 512;
        if(x2 > 0) x2 /= 15.03;
        else x2 /= 16.51;
        if(y2 > 0) y2 /= 15.03;
        else y2 /= 16.51;
        if(x2 > 31) x2 = 31;
        else if(x2 < (-31)) x2 = (-31);
        if(y2 > 31) y2 = 31;
        else if(y2 < (-31)) y2 = (-31);
    }
    x1 -= 512;
    y1 -= 512;// -512, 466
    if(x1 > 0) x1 /= (-15.03);
    else x1 /= (-16.5);
    if(y1 > 0) y1 /= 15.5;
    else y1 /= 16.5;
    if(x1 > 31) x1 = 31;
    else if(x1 < (-31)) x1 = (-31);
    if(y1 > 31) y1 = 31;
    else if(y1 < (-31)) y1 = (-31);
}

void interrupt ISR(){
    if(RCIF){
        receive = RCREG;
        if(OERR){
            CREN = 0;
            CREN = 1;
        }
    }
    if(TMR0IF){
        TMR0IF = 0;
        if(cursor == mode_dpad){
            send_byte(buttons_send);
        }
        if(cursor == mode_quad){
            if(mode_quad_counter % 2 == 0){
                if(button_Y == 0) throttle = 0;
                else if(button_R2 == 0)throttle++;
                else if(button_L2 == 0) throttle--;
                if(throttle < 0) throttle = 0;
                else if(throttle > 255) throttle = 255;
            }
            if(mode_quad_counter == 0){
                send_byte(throttle / 4);
                mode_quad_counter++;
            }
            else if(mode_quad_counter == 1){
                send_byte(buttons_send);
                mode_quad_counter++;
            }
            else if(mode_quad_counter == 2){
                send_byte(x1_send);
                mode_quad_counter++;
            }
            else if(mode_quad_counter == 3){
                mode_quad_counter = 0;
                send_byte(y1_send);
            }
        }
    }
}

void init(){
    TRISA = 0b00001111;
    TRISB = 0b00001100;
    TRISC = 0b00110110;
    TRISD = 0b11111111;
    TRISE = 0b00000000;
    ADON = 1;
    ADCON1 = 11;
    ADCON2 = 184;
    UCONbits.USBEN = 0;
    UCFGbits.UTRDIS = 0;
    i2c_init();
    USART_init(9600, 0);
}

void main(){
    float x_angle, y_angle;
    signed char j, t;
    int i;
    unsigned char flag = 1;
    receive = 0;
    init();
    rumble_left = 0;
    rumble_right = 0;
    delay_ms(200);
    //MPU6050_init();
    init_oled(0);
    init_oled(1);
    oled_clear_screen(0);
    oled_clear_screen(1);
    cursor = 0;
    set_xy(0, 0, 20);
    send_str(0, "Property of");
    set_xy(1, 0, 20);
    send_str(1, "Shivam Sharma");
    delay_ms(1000);
    oled_clear_screen(0);
    oled_clear_screen(1);
    set_xy(0, 0, 12);
    send_str(0, "Quad");
    set_xy(0, 1, 12);
    send_str(0, "D-Pad");
    set_xy(0, 2, 12);
    send_str(0, "Receiver");
    y1 = 0;
    y2 = 0;
    x1 = 0;
    x2 = 0;
    do{
        //get_adc_values(0);
        if((button_up == 0 || y1 > 25) && flag == 1){
            cursor++;
            flag = 0;
        }
        else if((button_down == 0 || y1 < (-25)) && flag == 1){
            cursor--;
            flag = 0;
        }
        else flag = 1;
        if(cursor > 2) cursor = 2;
        else if(cursor < 0) cursor = 0;
        set_xy(0, 0, 0);
        if(cursor == mode_quad) send_str(0, "->");
        else send_str(0, "  ");
        set_xy(0, 1, 0);
        if(cursor == mode_dpad)send_str(0, "->");
        else send_str(0, "  ");
        set_xy(0, 2, 0);
        if(cursor == mode_receiver)send_str(0, "->");
        else send_str(0, "  ");
    }while(button_A);
    oled_clear_screen(0);
    if(cursor == mode_quad){
        throttle = 0;
        c = 0;
        mode_quad_counter = 0;
        RCIE = 1;
        timer_init(6);
        while(1){
            get_adc_values(0);
            direction = 0;
            if(!button_A) direction += 1;
            if(!button_B) direction += 2;
            if(!button_X){
                direction += 4;
                throttle = 0;
            }
            if((throttle / 2) % 2 == 1) direction += 8;
            if(!button_R3) direction += 16;
            if(!button_L3) direction += 32;
            buttons_send = direction + 64;
            x1_send = 128 + (x1 + 31);
            y1_send = 192 + (y1 + 31);
            
            //Display
            set_xy(0, 0, 0);
            i2c_start();
            i2c_send(0x78);
            i2c_send(0x40);
            for(i = 0; i < 128; i++){
                if(i < c) i2c_send(255);
                else i2c_send(0);
            }
            i2c_stop();
            set_xy(1, 0, 0);
            i2c_start();
            i2c_send(0x7A);
            i2c_send(0x40);
            for(i = 0; i < 128; i++){
                if(i <= (throttle - 128)) i2c_send(255);
                else i2c_send(0);
            }
            i2c_stop();
            for(i = 0, j = -31; i < 4; i++){
                set_xy(0, (i + 4), 0);
                i2c_start();
                i2c_send(0x78);
                i2c_send(0x40);
                i2c_send(255);
                t = 0;
                if(y1 / 2 == j++) t = 0;
                if(y1 / 2 == j++) t = 1;
                if(y1 / 2 == j++) t = 2;
                if(y1 / 2 == j++) t = 4;
                if(y1 / 2 == j++) t = 16;
                if(y1 / 2 == j++) t = 32;
                if(y1 / 2 == j++) t = 64;
                if(y1 / 2 == j++) t = 128;
                i2c_send(t);
                i2c_send(t);
                i2c_send(t);
                i2c_send(t);
                i2c_send(t);
                i2c_send(t);
                i2c_send(255);
                i2c_stop();
            }
            set_xy(0, 7, 12);
            i2c_start();
            i2c_send(0x78);
            i2c_send(0x40);
            for(i = -31; i < 32; i++){
                if(i == x1) i2c_send(255);
                else i2c_send(129);
            }
            i2c_stop();
            set_xy(1, 2, 0);
            if((buttons_send % 2) == 1) send_char(1, 'A');
            else send_char(1, ' ');
            set_xy(1, 3, 0);
            if(((buttons_send / 2) % 2) == 1) send_char(1, 'B');
            else send_char(1, ' ');
            set_xy(1, 4, 0);
            if(((buttons_send / 4) % 2) == 1) send_char(1, 'X');
            else send_char(1, ' ');
            set_xy(1, 5, 0);
            if(((buttons_send / 8) % 2) == 1) send_char(1, 'Y');
            else send_char(1, ' ');
            set_xy(1, 6, 0);
            if(((buttons_send / 16) % 2) == 1) send_str(1, "R3");
            else send_str(1, "  ");
            set_xy(1, 7, 0);
            if(((buttons_send / 32) % 2) == 1) send_str(1, "L3");
            else send_str(1, "  ");
        }
    }
    else if(cursor == mode_dpad){
        RCIE = 1;
        timer_init(7);
        while(1){
            if(button_up == 0 || button_down == 0 || button_left == 0 || button_right == 0){
                if(button_up == 0 && button_down == 1 && button_left == 1 && button_right == 1) direction = 1;
                else if(button_up == 0 && button_down == 1 && button_left == 1 && button_right == 0) direction = 2;
                else if(button_up == 1 && button_down == 1 && button_left == 1 && button_right == 0) direction = 3;
                else if(button_up == 1 && button_down == 0 && button_left == 1 && button_right == 0) direction = 4;
                else if(button_up == 1 && button_down == 0 && button_left == 1 && button_right == 1) direction = 5;
                else if(button_up == 1 && button_down == 0 && button_left == 0 && button_right == 1) direction = 6;
                else if(button_up == 1 && button_down == 1 && button_left == 0 && button_right == 1) direction = 7;
                else if(button_up == 0 && button_down == 1 && button_left == 0 && button_right == 1) direction = 8;
                else direction = 0;
            }
            else{
                get_adc_values(0);
                if(x1 < 25 && x1 > (-25) && y1 > 25) direction = 1;
                else if(x1 > 25 && y1 > 25) direction = 2;
                else if(x1 > 25 && y1 < 25 && y1 > (-25)) direction = 3;
                else if(x1 > 25 && y1 < (-25)) direction = 4;
                else if(x1 < 25 && x1 > (-25) && y1 < (-25)) direction = 5;
                else if(x1 < (-25) && y1 < (-25)) direction = 6;
                else if(x1 < (-25) && y1 < 25 && y1 > (-25)) direction = 7;
                else if(x1 < (-25) && y1 > 25) direction = 8;
                else direction = 0;
            }
            if(!button_A){
                direction += 16;
            }
            if(!button_B){
                direction += 32;
            }
            if(!button_X){
                direction += 64;
            }
            if(!button_Y){
                direction += 128;
            }
            buttons_send = direction;
            set_xy(0, 0, 0);
            if((buttons_send % 16) == 1) send_str(0, "Up         ");
            else if((buttons_send % 16) == 2) send_str(0, "Up-Right   ");
            else if((buttons_send % 16) == 3) send_str(0, "Right      ");
            else if((buttons_send % 16) == 4) send_str(0, "Down-Right ");
            else if((buttons_send % 16) == 5) send_str(0, "Down       ");
            else if((buttons_send % 16) == 6) send_str(0, "Down-Left  ");
            else if((buttons_send % 16) == 7) send_str(0, "Left       ");
            else if((buttons_send % 16) == 8) send_str(0, "Up-Left    ");
            else send_str(0, "           ");
            set_xy(1, 0, 0);
            if(((buttons_send / 16) % 2) == 1) send_str(0, "A");
            else  send_str(0, " ");
            if(((buttons_send / 32) % 2) == 1) send_str(0, "B");
            else send_str(0, " ");
            if(((buttons_send / 64) % 2) == 1) send_str(0, "X");
            else  send_str(0, " ");
            if(((buttons_send / 128) % 2) == 1) send_str(0, "Y");
            else send_str(0, " ");
            set_xy(0, 2, 0);
            write_int(0, receive, 3);
        }
    }
    
    else if(cursor == mode_receiver){
        while(1){
            t = receive_byte();
            set_xy(0, 0, 0);
            write_int(0, t, 3);
        }
    }
}

