#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include <sys/attribs.h>
#include <string.h>

#include "10DOF.h"
#include "bitbang_i2c.h"
#include "draw.h"
#include "ili9341.h"
#include "inputdata.h"
#include "pic32.h"
#include "pragma.h"
#include "SPI.h"
#include "touchscreen.h"
#include "USART.h"

#include "settings.h"

#define HEADER_ID1 0x1A
#define HEADER_ID2 0x0A

#define FOOTER_ID1 0x2B
#define FOOTER_ID2 0xF1

void speaker_tone(float);
void adc_init();
void get_adc_values();
void get_touchscreen();

char serial_monitor[30][54];
char temp_sr_str[54];
unsigned char sr_line_len[30];
unsigned char pre_sr_line_len[30];

unsigned char receive1;
volatile char rx_buffer_global[1024];
int buffer_counter = 0;

unsigned int rx_time_counter = 0;
volatile bool rx_signal = 0, rx_data_rdy = 0;

int vibrate_timer = -1;

volatile unsigned char rx_stage = 0;
volatile unsigned int rx_len = 0;

void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4SRS) delay_timer(void) {
    IFS0bits.T2IF = 0;
    delay_counter++;
}

int main() {
    bool rx_signal_flag = 1;
    int i, j, k, c;

    unsigned char mode = 0;
    unsigned char pre_cursor = 100, pre_mode = 0;

    unsigned char data_len;
    float data[20];
    char data_names[20][40];
    
    unsigned char pre_d1, pre_d2;
    
    char rx_buffer[1024];
    unsigned int rx_buffer_len = 0;
    
    for(i = 0; i < 30; i++) {
        pre_sr_line_len[i] = 0;
    }
    
    init();
//    pwm_init(10000);
    TRISBbits.TRISB2 = 0;
    LATBbits.LATB2 = 0;
    adc_init();
    timer2_init(1000);
    timer3_init(50);
    timer5_init(1000);
//    USART1_init(111111);
    USART1_init(115200);
    SPI_init();
    
    speaker_pin = 0;
    LCD_backlight_pin = 0;    
    
    delay_ms(150);
    CS = 1;
    RST = 1;
    delay_ms(50);
    RST = 0;
    delay_ms(50);
    RST = 1;
    delay_ms(50);
    ColorLCD_init();
    delay_ms(120);     
    ColorLCD_writecommand(0x29);        //Display on  
    FillRect(0, 0, 320, 250, 0xFFFF);   //Clear screen   
    
    DrawDisplayBounds();
    FillRect(270, (float)(31 - analog2_y) * 240.0/31.0, 50, (float)(analog2_y) * 240.0/31.0, 0xFFA0);
    FillRect(270, 0, 50, (float)(31 - analog2_y) * 240.0/31.0,0xFFFF);
    
    pre_d1 = dial1;
    pre_d2 = dial2;
    
    while(1) {
        if(rx_data_rdy && rx_signal) {
            
            IEC3bits.U1RXIE = 0;
            for(i = 0; i < 1024; i++) {
                rx_buffer[i] = rx_buffer_global[i];
            }
            IEC3bits.U1RXIE = 1;
            
//            memcpy(rx_buffer, rx_buffer_global, 1024);
            
            for(rx_buffer_len = 0; rx_buffer_len < 1024 && rx_buffer[rx_buffer_len] != '\0'; rx_buffer_len++);
            
            mode = rx_buffer[0];
            if(mode == 'F') {
                data_len = DecodeStringF(rx_buffer, data_names, data);
            } else {
                if(DecodeString(rx_buffer, data) == 0) {
                    rx_data_rdy = 0;
                    continue;
                }
            }
            rx_data_rdy = 0;
            rx_signal_flag = 1;
            
            if(mode != 'Z') {
                //--------------------------------New mode: clear screen and display new constants for that mode---------------------------
                if(mode != pre_mode){
                    FillRect(0, 8, 250, 15 * 8, 0xFFFF);
                    if(mode == 'A') {
                        WriteStr("  P:", 0, 1 * 8, 0x0000);
                        WriteStr("  I:", 0, 2 * 8, 0x0000);
                        WriteStr("  D:", 0, 3 * 8, 0x0000);
                        WriteStr("  Altitude-p:", 0, 4 * 8, 0x0000);
                        WriteStr("  Altitude-i:", 0, 5 * 8, 0x0000);
                        WriteStr("  Altitude-d:", 0, 6 * 8, 0x0000);
                        WriteStr("  GPS connected:", 0, 7 * 8, 0x0000);
                        WriteStr("  GPS signal:", 0, 8 * 8, 0x0000);
                        FillRect(80, 9, 62, 10, 0xF000);
                        FillRect(81, 10, 60, 8, 0xFFFF);
                    }
                    else if(mode == 'B') {
                        WriteStr(" Arming:", 0, 1 * 8, 0x0000);
                        FillRect(1, 19, 152, 17, 0xF000);
                        FillRect(2, 20, 150, 15, 0xFFFF);
                    }
                    else if(mode == 'C') {
                        WriteStr(" Mode:", 0, 2 * 8, 0x0000);
                        WriteStr(" Pitch:", 0, 2 * 8, 0x0000);
                        WriteStr(" Roll:", 0, 3 * 8, 0x0000);
                        WriteStr(" Yaw:", 0, 4 * 8, 0x0000);
                        WriteStr(" Altitude:", 0, 5 * 8, 0x0000);
                        WriteStr(" Latitude:", 0, 6 * 8, 0x0000);
                        WriteStr(" Longitude:", 0, 7 * 8, 0x0000);
                    } 
                    else if(mode == 'D') {
                        WriteStr("Calibration",    0, 0,    0xF80F);
                        WriteStr("Acc x:",         0, 1 *8, 0xF80F);
                        WriteStr("Acc y:",         0, 2 *8, 0xF80F);
                        WriteStr("Acc z:",         0, 3 *8, 0xF80F);
                        WriteStr("Gyro x:",        0, 4 *8, 0xF80F);
                        WriteStr("Gyro y:",        0, 5 *8, 0xF80F);
                        WriteStr("Gyro z:",        0, 6 *8, 0xF80F);
                        WriteStr("Compass x:",     0, 7 *8, 0xF80F);
                        WriteStr("Compass y:",     0, 8 *8, 0xF80F);
                        WriteStr("Compass z:",     0, 9 *8, 0xF80F);
                        WriteStr("Compass x Max:", 0, 10*8, 0xF80F);
                        WriteStr("Compass y Max:", 0, 11*8, 0xF80F);
                        WriteStr("Compass z Max:", 0, 12*8, 0xF80F);
                        WriteStr("Compass x Min:", 0, 13*8, 0xF80F);
                        WriteStr("Compass y Min:", 0, 14*8, 0xF80F);
                        WriteStr("Compass z Min:", 0, 15*8, 0xF80F);
                    }
                }
                pre_mode = mode;

                //--------------------------------------------Display variables for the current mode---------------------------------------

                if(mode == 'A') {
                    if((int)data[0] != pre_cursor) {
                        WriteStr(" ", 4, (pre_cursor + 1) * 8, 0x0000);
                        WriteStr(">", 4, ((int)data[0] + 1) * 8, 0x0000);
                    }
                    pre_cursor = (int)data[0];
                    WriteFloat(data[1], 3, 3, 5  * 6, 1 * 8, 0x0000);
                    WriteFloat(data[2], 3, 3, 5  * 6, 2 * 8, 0x0000);
                    WriteFloat(data[3], 3, 3, 5  * 6, 3 * 8, 0x0000);
                    WriteFloat(data[4], 3, 3, 14 * 6, 4 * 8, 0x0000);
                    WriteFloat(data[5], 3, 3, 14 * 6, 5 * 8, 0x0000);
                    WriteFloat(data[6], 3, 3, 14 * 6, 6 * 8, 0x0000);
                    if((int)data[7]) 
                        WriteStr("Yes", 17 * 6, 7 * 8, 0x0000);
                    else 
                        WriteStr("No ", 17 * 6, 7 * 8, 0x0000);
                    
                    if((int)data[8]) 
                        WriteStr("Yes", 14 * 6, 8 * 8, 0x0000);
                    else 
                        WriteStr("No ", 14 * 6, 8 * 8, 0x0000);
                    
                    if((int)data[9]) 
                        FillRect(81, 10, (int)data[9] * 3, 8, 0xFFA0);
                    else 
                        FillRect(81, 10, 60, 8, 0xFFFF);
                }

                else if(mode == 'B') {
                    FillRect(2, 20, ((int)data[0] / 10) + 1, 15, 0xFFA0);
                }

                else if(mode == 'C') {
                    unsigned char loop_mode = rx_buffer[rx_buffer_len - 1];
                    if(loop_mode == '1')       WriteStr("RC      ", 7 * 6, 1 * 8, 0xF80F);
                    else if(loop_mode == '2')  WriteStr("Alt-hold", 7 * 6, 1 * 8, 0xF80F);
                    else if(loop_mode == '3')  WriteStr("Pos-hold", 7 * 6, 1 * 8, 0xF80F);
                    else if(loop_mode == '0')  WriteStr("Killed  ", 7 * 6, 1 * 8, 0xF80F);
                    WriteFloat(data[0], 3, 2, 8  * 6, 2 * 8, 0x0000);
                    WriteFloat(data[1], 3, 2, 7  * 6, 3 * 8, 0x0000);
                    WriteFloat(data[2], 3, 2, 6  * 6, 4 * 8, 0x0000);
                    WriteFloat(data[3], 3, 2, 11 * 6, 5 * 8, 0x0000);
                    WriteFloat(data[4], 3, 6, 11 * 6, 6 * 8, 0x0000);
                    WriteFloat(data[5], 3, 6, 12 * 6, 7 * 8, 0x0000);
                }

                /*else if(mode == 'F') {
                    for(i = 0; i < data_len; i++) {
                        if(!strcasecmp(data_names[i], "loop mode"))
                            break;
                    }
                    if(i >= data_len)
                        break;

                    if((int)data[i] == 1)       WriteStr("RC      ", 7 * 6, 1 * 8, 0xF80F);
                    else if((int)data[i] == 2)  WriteStr("Alt-hold", 7 * 6, 1 * 8, 0xF80F);
                    else if((int)data[i] == 3)  WriteStr("Pos-hold", 7 * 6, 1 * 8, 0xF80F);
                    else if((int)data[i] == 0)  WriteStr("Killed  ", 7 * 6, 1 * 8, 0xF80F);

                    for(j = 0; j < data_len; j++) {
                        if(i != j) {
                            WriteStr(data_names[j], 0, (j+2)*8, 0xF80F);
                            WriteStr(": ", strlen(data_names[j])*6, (j+2)*8, 0xF80F);
                            WriteFloat(data[j], 3, 3, (strlen(data_names[j])+2)*6, (j+2) * 8, 0x0000);
                        }
                    }
                }*/

                else if(mode == 'D') {
                    WriteInt(data[0],   6, 7 *6, 1 *8, 0x0000);
                    WriteInt(data[1],   6, 7 *6, 2 *8, 0x0000);
                    WriteInt(data[2],   6, 7 *6, 3 *8, 0x0000);
                    WriteInt(data[3],   6, 8 *6, 4 *8, 0x0000);
                    WriteInt(data[4],   6, 8 *6, 5 *8, 0x0000);
                    WriteInt(data[5],   6, 8 *6, 6 *8, 0x0000);
                    WriteInt(data[6],   6, 11*6, 7 *8, 0x0000);
                    WriteInt(data[7],   6, 11*6, 8 *8, 0x0000);
                    WriteInt(data[8],   6, 11*6, 9 *8, 0x0000);
                    WriteInt(data[9],   6, 15*6, 10*8, 0x0000);
                    WriteInt(data[10],  6, 15*6, 11*8, 0x0000);
                    WriteInt(data[11],  6, 15*6, 12*8, 0x0000);
                    WriteInt(data[12],  6, 15*6, 13*8, 0x0000);
                    WriteInt(data[13],  6, 15*6, 14*8, 0x0000);
                    WriteInt(data[14],  6, 15*6, 15*8, 0x0000);
                }
                
                //ShowInputData();
            }

            //------------------------------------------------------Serial Monitor mode-------------------------------------------------
            
            else {
                if(pre_mode != mode) {
                    FillRect(0, 0, 320, 250, 0xFFFF);
                    WriteStr("Serial Monitor", 0, 0, 0xF80F);
                }
                pre_mode = mode;
                
                for(i = 0, j = 0, c = 1; rx_buffer[c] != '\0'; c++) {
                    if(rx_buffer[c] == '\n') {
                        serial_monitor[j][i] = '\0';
                        i = 0;
                        j++;
                    } else {
                        serial_monitor[j][i++] = rx_buffer[c];
                    }
                }
                serial_monitor[j][i] = '\0';

                for(i = 0; i <= j; i++)
                    sr_line_len[i] = strlen(serial_monitor[i]);
                for(; i < 30; i++)
                    sr_line_len[i] = 0;

                for(i = 0; i < 30; i++) {
                    if(pre_sr_line_len[i] > sr_line_len[i]) {
                        for(k = 0; k < pre_sr_line_len[i]; k++) {
                            temp_sr_str[k] = ' ';
                        }
                        temp_sr_str[k] = '\0';
                        WriteStr(temp_sr_str, 0, (i + 1) * 8, 0xFFFF);
                    }
                }

                for(i = 0; i <= j; i++) {
                    WriteStr(serial_monitor[i], 0, (i + 1) * 8, 0x0000);
                }

                for(i = 0; i < 30; i++) {
                    pre_sr_line_len[i] = sr_line_len[i];
                }
            }   
        }
        else if(!rx_signal && rx_signal_flag) {
            FillRect(0, 8, 250, 9 * 8, 0xFFFF);
            DrawDisplayBounds();
            rx_signal_flag = 0;
            pre_mode = 0;
        }
        
        if(pre_d1 != dial1 || pre_d2 != dial2) {
//            write_pwm(1, 255);
            LATBbits.LATB2 = 1;
            vibrate_timer = 150; //350
        }
        pre_d1 = dial1;
        pre_d2 = dial2;
        
        ShowInputData();
    }
    
    return 0;
}

void __ISR_AT_VECTOR(_UART1_RX_VECTOR, IPL6SRS) Xbee(void) {
    IFS3bits.U1RXIF = 0; 
//    do {
//        receive1 = U1RXREG & 0xFF;
//        if(receive1 == '\r') {
//            rx_signal = 1;
//            rx_data_rdy = 1;
//            rx_time_counter = 0;
//            
//            rx_buffer_global[buffer_counter] = '\0';
//            buffer_counter = 0;
//        } else {
//            rx_buffer_global[buffer_counter++] = receive1;
//        }
//    }while(U1STAbits.URXDA);
    static int temp_rx_len;
    do {
        receive1 = U1RXREG & 0xFF;
        if(receive1 == '\f') {
            rx_stage = 1;
        } else if(rx_stage == 1) {
            rx_len = (receive1 - '0') * 100;
            rx_stage = 2;
        } else if(rx_stage == 2) {
            rx_len += (receive1 - '0') * 10;
            rx_stage = 3;
        } else if(rx_stage == 3) {
            rx_len += (receive1 - '0');
            if(rx_len > 999) {
                rx_stage = 0;
            }
            temp_rx_len = rx_len;
            rx_stage = 4;
            buffer_counter = 0;
        } else if(rx_stage == 4 && temp_rx_len > 1) {
            rx_buffer_global[buffer_counter++] = receive1;
            temp_rx_len--;
        } else if(rx_stage == 4 && temp_rx_len <= 1) {
            rx_buffer_global[buffer_counter++] = receive1;
            rx_buffer_global[buffer_counter] = '\0';
            rx_signal = 1;
            rx_data_rdy = 1;
            rx_time_counter = 0;
            rx_stage = 0;
        }
    }while(U1STAbits.URXDA);    
    
    U1STAbits.OERR = 0;
    IFS3bits.U1RXIF = 0; 
}

void __ISR_AT_VECTOR(_TIMER_5_VECTOR, IPL4SRS) XBee_rx(void) {
    IFS0bits.T5IF = 0;
    if(rx_time_counter >= 1000) {
        rx_signal = 0;
        rx_data_rdy = 0;
    } else {
        rx_time_counter++;
    }    
    
    if(vibrate_timer >= 0) {
        if(vibrate_timer-- <= 0) {
//            write_pwm(1, 0);
            LATBbits.LATB2 = 0;
            vibrate_timer = -1;
        }
    }
}

void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL3SRS) Xbee_send(void){
    IFS0bits.T3IF = 0;
    get_adc_values();
    switch1 = switch1_pin;
    switch2 = switch2_pin;
    
    U1TXREG = (analog1_x + 15);
    U1TXREG = (32 + (analog1_y + 15));
    U1TXREG = (64 + (analog2_x + 15));
    U1TXREG = (96 + analog2_y);
    U1TXREG = (128 + !switch1 * 2 + !switch2);
    U1TXREG = (160 + dial2 * 4 + dial1);
}

unsigned int speaker_counter = 0;
unsigned long int tracker = 0;

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL7SRS) speaker_timer(void){
    IFS0bits.T4IF = 0;
    //LATCINV = 0x4000;
    //speaker_pin = !speaker_pin;
}

void get_adc_values(){
    int i;
    analog1_x = 0;
    analog1_y = 0;
    analog2_x = 0;
    analog2_y = 0;
    for(i = 0; i < 10; i++){
        ADCCON3bits.GSWTRG = 1;
        while(ADCDSTAT1bits.ARDY3 == 0 || ADCDSTAT1bits.ARDY4 == 0 || ADCDSTAT1bits.ARDY9 == 0 || ADCDSTAT1bits.ARDY10 == 0 || ADCDSTAT1bits.ARDY12 == 0 || ADCDSTAT1bits.ARDY13 == 0);
        analog1_x += ADCDATA12;
        analog2_x += ADCDATA9;
        analog1_y += ADCDATA13;
        analog2_y += ADCDATA10;
        dial1 += ADCDATA3;
        dial2 += ADCDATA4;
    }
    dial1 /= 10;
    dial2 /= 10;
    
    if(dial1 < 1240) {
        dial1 = 2;
    } else if(dial1 < 2000) { 
        dial1 = 1;
    } else {
        dial1 = 0;
    }
    
    if(dial2 < 1240) {
        dial2 = 0;
    }else if(dial2 < 2000) {
        dial2 = 1;
    } else {
        dial2 = 2;
    }
    
//    dial1 = 0;
//    dial2 = 0;
    
    analog1_x = 31.5 * (analog1_x - x1_offset) / (x1_max - x1_min);
    analog1_y = 31.5 * (analog1_y - y1_offset) / (y1_max - y1_min);
    analog2_x = 31.5 * (analog2_x - x2_offset) / (x2_max - x2_min);
    analog2_y = 31.5 * (analog2_y - y2_min) / (y2_max - y2_min);
    if(analog1_x > 15) analog1_x = 15;
    else if(analog1_x < (-15)) analog1_x = (-15);
    if(analog1_y > 15) analog1_y = 15;
    else if(analog1_y < (-15)) analog1_y = (-15);
    if(analog2_x > 15) analog2_x = 15;
    else if(analog2_x < (-15)) analog2_x = (-15);
}

void speaker_tone(float frequency){
    float t = 100000000.0 / frequency; unsigned char pre = 0;
    while(t > 65535){ t /= 2.0; pre++; }
    t = (int)t;
    while((int)t % 2 == 0 && pre < 8){ t /= 2.0; pre++; }
    if(pre == 7){ t *= 2.0; pre--; }
    if(pre == 8) pre = 7;
    T4CONbits.TCKPS = pre;
    PR4 = (int)t - 1;
    //TMR4 = 0;
}