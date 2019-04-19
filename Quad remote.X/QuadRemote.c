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
#include "timer_ISR.h"
#include "touchscreen.h"
#include "USART.h"

#include "settings.h"

void speaker_tone(float);
void adc_init();
void get_adc_values();
void get_touchscreen();

volatile char serial_monitor[30][54];

unsigned char receive1;
char rx_buffer[200];
int buffer_counter = 0;

unsigned int rx_time_counter = 0;
bool rx_signal = 0;

char mode = 0;
float pid_p = 0, pid_i = 0, pid_d = 0, altitude_p, altitude_i, altitude_d;          //Mode 'A'
unsigned char cursor = 0, arming_counter = 0, GPS_signal = 0, GPS_connected = 0;    //Mode 'A'
int arming_time;                                                                    //Mode 'B'
float pitch, roll, yaw, altitude, latitude, longitude, loop_mode;                   //Mode 'C'
int acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, compass_x, compass_y, compass_z;   //Mode 'D' 
int compass_max_x, compass_max_y, compass_max_z, compass_min_x, compass_min_y, compass_min_z; 

void main() {
    bool rx_signal_flag = 1;
    int i, j;
    unsigned char pre_cursor = 100, pre_mode = 0;
    
    char monitor[30][54], pre_monitor[30][54];
    
    init();
    adc_init();
    timer2_init(1000);
    timer3_init(50);
    timer5_init(1000);
    USART1_init(111111);
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
    
    while(1) {
        if(mode == 0 && rx_signal_flag == 1) {
            FillRect(0, 8, 250, 9 * 8, 0xFFFF);
            rx_signal_flag = 0;
        }
        else if(mode != 0 && rx_signal == 0) {
            DrawDisplayBounds();
        }
        if(mode != 0) {
            rx_signal_flag = 1;
        }
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
                    FillRect(1, 19, 102, 17, 0xF000);
                    FillRect(2, 20, 100, 15, 0xFFFF);
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
                    WriteStr("Calibration", 0, 0, 0xF80F);
                    WriteStr("Acc x:", 0, 1*8, 0xF80F);
                    WriteStr("Acc y:", 0, 2*8, 0xF80F);
                    WriteStr("Acc z:", 0, 3*8, 0xF80F);
                    WriteStr("Gyro x:", 0, 4*8, 0xF80F);
                    WriteStr("Gyro y:", 0, 5*8, 0xF80F);
                    WriteStr("Gyro z:", 0, 6*8, 0xF80F);
                    WriteStr("Compass x:", 0, 7*8, 0xF80F);
                    WriteStr("Compass y:", 0, 8*8, 0xF80F);
                    WriteStr("Compass z:", 0, 9*8, 0xF80F);
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
                if(cursor != pre_cursor) {
                    WriteStr(" ", 4, (pre_cursor + 1) * 8, 0x0000);
                    WriteStr(">", 4, (cursor + 1) * 8, 0x0000);
                }
                pre_cursor = cursor;
                WriteFloat(pid_p, 2, 2, 5 * 6, 1 * 8, 0x0000);
                WriteFloat(pid_i, 2, 2, 5 * 6, 2 * 8, 0x0000);
                WriteFloat(pid_d, 2, 2, 5 * 6, 3 * 8, 0x0000);
                WriteFloat(altitude_p, 3, 1, 14 * 6, 4 * 8, 0x0000);
                WriteFloat(altitude_i, 2, 2, 14 * 6, 5 * 8, 0x0000);
                WriteFloat(altitude_d, 3, 1, 14 * 6, 6 * 8, 0x0000);
                if(GPS_connected) WriteStr("Yes", 17 * 6, 7 * 8, 0x0000);
                else WriteStr("No ", 17 * 6, 7 * 8, 0x0000);
                if(GPS_signal) WriteStr("Yes", 14 * 6, 8 * 8, 0x0000);
                else WriteStr("No ", 14 * 6, 8 * 8, 0x0000);
                if(arming_counter) FillRect(81, 10, arming_counter * 3, 8, 0xFFA0);
                else FillRect(81, 10, 60, 8, 0xFFFF);
            }

            if(mode == 'B') {
                FillRect(2, 20, (arming_time / 10) + 1, 15, 0xFFA0);
            }

            if(mode == 'C') {
                if(loop_mode == 'S') WriteStr("RC      ", 7 * 6, 1 * 8, 0xF80F);
                else if(loop_mode == 'A') WriteStr("Alt-hold", 7 * 6, 1 * 8, 0xF80F);
                else if(loop_mode == 'P') WriteStr("Pos-hold", 7 * 6, 1 * 8, 0xF80F);
                else if(loop_mode == 'K') WriteStr("Killed  ", 7 * 6, 1 * 8, 0xF80F);
                WriteFloat(pitch, 3, 2, 8 * 6, 2 * 8, 0x0000);
                WriteFloat(roll, 3, 2, 7 * 6, 3 * 8, 0x0000);
                WriteFloat(yaw, 3, 2, 6 * 6, 4 * 8, 0x0000);
                WriteFloat(altitude, 3, 2, 11 * 6, 5 * 8, 0x0000);
                WriteFloat(latitude, 3, 6, 11 * 6, 6 * 8, 0x0000);
                WriteFloat(longitude, 3, 6, 12 * 6, 7 * 8, 0x0000);
            }

            if(mode == 'D') {
                WriteInt(acc_x, 6, 7*6, 1*8, 0x0000);
                WriteInt(acc_y, 6, 7*6, 2*8, 0x0000);
                WriteInt(acc_z, 6, 7*6, 3*8, 0x0000);
                WriteInt(gyro_x, 6, 8*6, 4*8, 0x0000);
                WriteInt(gyro_y, 6, 8*6, 5*8, 0x0000);
                WriteInt(gyro_z, 6, 8*6, 6*8, 0x0000);
                WriteInt(compass_x, 6, 11*6, 7*8, 0x0000);
                WriteInt(compass_y, 6, 11*6, 8*8, 0x0000);
                WriteInt(compass_z, 6, 11*6, 9*8, 0x0000);
                WriteInt(compass_min_x, 6, 15*6, 10*8, 0x0000);
                WriteInt(compass_min_y, 6, 15*6, 11*8, 0x0000);
                WriteInt(compass_min_z, 6, 15*6, 12*8, 0x0000);
                WriteInt(compass_max_x, 6, 15*6, 13*8, 0x0000);
                WriteInt(compass_max_y, 6, 15*6, 14*8, 0x0000);
                WriteInt(compass_max_z, 6, 15*6, 15*8, 0x0000);
            }
            
            ShowInputData();
        }

        //------------------------------------------------------Serial Monitor mode-------------------------------------------------

        else {
            if(pre_mode != mode) FillRect(0, 0, 320, 250, 0xFFFF);
            pre_mode = mode;
            
            for(j = 0; j < 30; j++) {
                for(i = 0; serial_monitor[j][i] != '\0'; i++) {
                    monitor[j][i] = serial_monitor[j][i];
                }
                monitor[j][i] = '\0';
            }
            
            for(i = 0; i < 30; i++) {
                if(monitor[i][0] != '\0' && strcmp(monitor[i], pre_monitor[i])) WriteStr(monitor[i], 0, i * 8, 0x0000);
            }
            
            for(i = 0; i < 30; i++) {
                strcpy(monitor[i], pre_monitor[i]);
            }
        }    
    }
}

void __ISR_AT_VECTOR(_UART1_RX_VECTOR, IPL6SRS) Xbee(void){
    int i, j;
    IFS3bits.U1RXIF = 0; 
    do {
        receive1 = U1RXREG & 0xFF;
        if(receive1 == '\r') {
            rx_signal = 1;
            rx_time_counter = 0;
            
            rx_buffer[buffer_counter] = '\0';
            buffer_counter = 0;
            if(rx_buffer[0] > 64 && rx_buffer[0] < 91) {
                mode = rx_buffer[0];
                if(mode == 'A') {
                    cursor = rx_buffer[1] - 48;
                    pid_p = 10 * (float)(rx_buffer[3] - 48) + (float)(rx_buffer[4] - 48) + 0.1 * (float)(rx_buffer[6] - 48) + 0.01 * (float)(rx_buffer[7] - 48);
                    if(rx_buffer[2] == '-') pid_p *= (-1);
                    pid_i= 10 * (float)(rx_buffer[9] - 48) + (float)(rx_buffer[10] - 48) + 0.1 * (float)(rx_buffer[12] - 48) + 0.01 * (float)(rx_buffer[13] - 48);
                    if(rx_buffer[8] == '-') pid_i *= (-1);
                    pid_d = 10 * (float)(rx_buffer[15] - 48) + (float)(rx_buffer[16] - 48) + 0.1 * (float)(rx_buffer[18] - 48) + 0.01 * (float)(rx_buffer[19] - 48);
                    if(rx_buffer[14] == '-') pid_d *= (-1);
                    altitude_p = 100 * (float)(rx_buffer[21] - 48) + 10 * (float)(rx_buffer[22] - 48) + (float)(rx_buffer[23] - 48) + 0.1 * (float)(rx_buffer[25] - 48);
                    if(rx_buffer[20] == '-') altitude_p *= (-1);
                    altitude_i = 10 * (float)(rx_buffer[27] - 48) + (float)(rx_buffer[28] - 48) + 0.1 * (float)(rx_buffer[30] - 48) + 0.01 * (float)(rx_buffer[31] - 48);
                    if(rx_buffer[26] == '-') altitude_i *= (-1);
                    altitude_d = 100 * (float)(rx_buffer[33] - 48) + 10 * (float)(rx_buffer[34] - 48) + (float)(rx_buffer[35] - 48) + 0.1 * (float)(rx_buffer[37] - 48);
                    if(rx_buffer[32] == '-') altitude_d *= (-1);
                    GPS_signal = rx_buffer[38] - 48;
                    GPS_connected = rx_buffer[39] - 48;
                    arming_counter = 10 * (rx_buffer[40] - 48) + (rx_buffer[41] - 48);
                }
                else if(mode == 'B'){
                    arming_time = (100 * (rx_buffer[1] - 48) + 10 * (rx_buffer[2] - 48) + (rx_buffer[3] - 48));
                }
                else if(mode == 'C'){
                    roll = 100 * (float)(rx_buffer[2] - 48) + 10 * (float)(rx_buffer[3] - 48) + (float)(rx_buffer[4] - 48) + 0.1 * (float)(rx_buffer[6] - 48) + 0.01 * (float)(rx_buffer[7] - 48);
                    if(rx_buffer[1] == '-') roll *= (-1);
                    pitch = 100 * (float)(rx_buffer[9] - 48) + 10 * (float)(rx_buffer[10] - 48) + (float)(rx_buffer[11] - 48) + 0.1 * (float)(rx_buffer[13] - 48) + 0.01 * (float)(rx_buffer[14] - 48);
                    if(rx_buffer[8] == '-') pitch *= (-1);
                    yaw = 100 * (float)(rx_buffer[16] - 48) + 10 * (float)(rx_buffer[17] - 48) + (float)(rx_buffer[18] - 48) + 0.1 * (float)(rx_buffer[20] - 48) + 0.01 * (float)(rx_buffer[21] - 48);
                    if(rx_buffer[15] == '-') yaw *= (-1);
                    altitude = 100 * (float)(rx_buffer[23] - 48) + 10 * (float)(rx_buffer[24] - 48) + (float)(rx_buffer[25] - 48) + 0.1 * (float)(rx_buffer[27] - 48) + 0.01 * (float)(rx_buffer[28] - 48);
                    if(rx_buffer[22] == '-') altitude *= (-1);
                    latitude = 100 * (float)(rx_buffer[30] - 48) + 10 * (float)(rx_buffer[31] - 48) + (float)(rx_buffer[32] - 48) + 0.1 * (float)(rx_buffer[34] - 48) + 0.01 * (float)(rx_buffer[35] - 48) + 0.001 * (float)(rx_buffer[36] - 48) + 0.0001 * (float)(rx_buffer[37] - 48) + 0.00001 * (float)(rx_buffer[38] - 48) + 0.000001 * (float)(rx_buffer[39] - 48) + 0.0000001 * (float)(rx_buffer[40] - 48) + 0.00000001 * (float)(rx_buffer[41] - 48);
                    if(rx_buffer[29] == '-') latitude *= (-1);
                    longitude = 100 * (float)(rx_buffer[43] - 48) + 10 * (float)(rx_buffer[44] - 48) + (float)(rx_buffer[45] - 48) + 0.1 * (float)(rx_buffer[47] - 48) + 0.01 * (float)(rx_buffer[48] - 48) + 0.001 * (float)(rx_buffer[49] - 48) + 0.0001 * (float)(rx_buffer[50] - 48) + 0.00001 * (float)(rx_buffer[51] - 48) + 0.000001 * (float)(rx_buffer[52] - 48) + 0.0000001 * (float)(rx_buffer[53] - 48) + 0.00000001 * (float)(rx_buffer[54] - 48);
                    if(rx_buffer[42] == '-') longitude *= (-1);
                    loop_mode = rx_buffer[55];
                }
                else if(mode == 'D'){
                    acc_x = 100000*(rx_buffer[2] - 48) + 10000*(rx_buffer[3] - 48) + 1000*(rx_buffer[4] - 48) + 100*(rx_buffer[5] - 48) + 10*(rx_buffer[6] - 48) + (rx_buffer[7] - 48);
                    if(rx_buffer[1] == '-') acc_x *= (-1);
                    acc_y = 100000*(rx_buffer[9] - 48) + 10000*(rx_buffer[10] - 48) + 1000*(rx_buffer[11] - 48) + 100*(rx_buffer[12] - 48) + 10*(rx_buffer[13] - 48) + (rx_buffer[14] - 48);
                    if(rx_buffer[8] == '-') acc_y *= (-1);
                    acc_z = 100000*(rx_buffer[16] - 48) + 10000*(rx_buffer[17] - 48) + 1000*(rx_buffer[18] - 48) + 100*(rx_buffer[19] - 48) + 10*(rx_buffer[20] - 48) + (rx_buffer[21] - 48);
                    if(rx_buffer[15] == '-') acc_z *= (-1);
                    
                    gyro_x = 100000*(rx_buffer[23] - 48) + 10000*(rx_buffer[24] - 48) + 1000*(rx_buffer[25] - 48) + 100*(rx_buffer[26] - 48) + 10*(rx_buffer[27] - 48) + (rx_buffer[28] - 48);
                    if(rx_buffer[22] == '-') gyro_x *= (-1);
                    gyro_y = 100000*(rx_buffer[30] - 48) + 10000*(rx_buffer[31] - 48) + 1000*(rx_buffer[32] - 48) + 100*(rx_buffer[33] - 48) + 10*(rx_buffer[34] - 48) + (rx_buffer[35] - 48);
                    if(rx_buffer[29] == '-') gyro_y *= (-1);
                    gyro_z = 100000*(rx_buffer[37] - 48) + 10000*(rx_buffer[38] - 48) + 1000*(rx_buffer[39] - 48) + 100*(rx_buffer[40] - 48) + 10*(rx_buffer[41] - 48) + (rx_buffer[42] - 48);
                    if(rx_buffer[36] == '-') gyro_z *= (-1);
                    
                    compass_x = 100000*(rx_buffer[44] - 48) + 10000*(rx_buffer[45] - 48) + 1000*(rx_buffer[46] - 48) + 100*(rx_buffer[47] - 48) + 10*(rx_buffer[48] - 48) + (rx_buffer[49] - 48);
                    if(rx_buffer[43] == '-') compass_x *= (-1);
                    compass_y = 100000*(rx_buffer[51] - 48) + 10000*(rx_buffer[52] - 48) + 1000*(rx_buffer[53] - 48) + 100*(rx_buffer[54] - 48) + 10*(rx_buffer[55] - 48) + (rx_buffer[56] - 48);
                    if(rx_buffer[50] == '-') compass_y *= (-1);
                    compass_z = 100000*(rx_buffer[58] - 48) + 10000*(rx_buffer[59] - 48) + 1000*(rx_buffer[60] - 48) + 100*(rx_buffer[61] - 48) + 10*(rx_buffer[62] - 48) + (rx_buffer[63] - 48);
                    if(rx_buffer[57] == '-') compass_z *= (-1);
                    
                    compass_min_x = 100000*(rx_buffer[65] - 48) + 10000*(rx_buffer[66] - 48) + 1000*(rx_buffer[67] - 48) + 100*(rx_buffer[68] - 48) + 10*(rx_buffer[69] - 48) + (rx_buffer[70] - 48);
                    if(rx_buffer[64] == '-') compass_min_x *= (-1);
                    compass_min_y = 100000*(rx_buffer[72] - 48) + 10000*(rx_buffer[73] - 48) + 1000*(rx_buffer[74] - 48) + 100*(rx_buffer[75] - 48) + 10*(rx_buffer[76] - 48) + (rx_buffer[77] - 48);
                    if(rx_buffer[71] == '-') compass_min_y *= (-1);
                    compass_min_z = 100000*(rx_buffer[79] - 48) + 10000*(rx_buffer[80] - 48) + 1000*(rx_buffer[81] - 48) + 100*(rx_buffer[82] - 48) + 10*(rx_buffer[83] - 48) + (rx_buffer[84] - 48);
                    if(rx_buffer[78] == '-') compass_min_z *= (-1);
                    
                    compass_max_x = 100000*(rx_buffer[86] - 48) + 10000*(rx_buffer[87] - 48) + 1000*(rx_buffer[88] - 48) + 100*(rx_buffer[89] - 48) + 10*(rx_buffer[90] - 48) + (rx_buffer[91] - 48);
                    if(rx_buffer[85] == '-') compass_max_x *= (-1);
                    compass_max_y = 100000*(rx_buffer[93] - 48) + 10000*(rx_buffer[94] - 48) + 1000*(rx_buffer[95] - 48) + 100*(rx_buffer[96] - 48) + 10*(rx_buffer[97] - 48) + (rx_buffer[98] - 48);
                    if(rx_buffer[92] == '-') compass_max_y *= (-1);
                    compass_max_z = 100000*(rx_buffer[100] - 48) + 10000*(rx_buffer[101] - 48) + 1000*(rx_buffer[102] - 48) + 100*(rx_buffer[103] - 48) + 10*(rx_buffer[104] - 48) + (rx_buffer[105] - 48);
                    if(rx_buffer[99] == '-') compass_max_z *= (-1);
                }
                else if(mode == 'Z') {
                    for(j = 0; j < 30; j++) {
                        serial_monitor[j][0] = '\0';
                    }
                    for(i = 0, j = 0, buffer_counter = 1; rx_buffer[buffer_counter] != '\0'; buffer_counter++) {
                        if(rx_buffer[buffer_counter] == '\n') {
                            serial_monitor[j][i] = '\0';
                            j++;
                            i = 0;
                        } else {
                            serial_monitor[j][i] = rx_buffer[buffer_counter];
                            i++;
                        }
                    }
                    serial_monitor[j][i] = '\0';
                    buffer_counter = 0;
                }
            }
        } else {
            rx_buffer[buffer_counter++] = receive1;
        }
    }while(U1STAbits.URXDA);
    U1STAbits.OERR = 0;
    IFS3bits.U1RXIF = 0; 
}

void __ISR_AT_VECTOR(_TIMER_5_VECTOR, IPL4SRS) XBee_rx(void) {
    IFS0bits.T5IF = 0;
    if(rx_time_counter >= 1000) {
        rx_signal = 0;
        mode = 0;
    } else {
        rx_time_counter++;
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
        while(ADCDSTAT1bits.ARDY9 == 0 || ADCDSTAT1bits.ARDY10 == 0 || ADCDSTAT1bits.ARDY12 == 0 || ADCDSTAT1bits.ARDY13 == 0);
        analog1_x += ADCDATA12;
        analog2_x += ADCDATA9;
        analog1_y += ADCDATA13;
        analog2_y += ADCDATA10;
    }
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