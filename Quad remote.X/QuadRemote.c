#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include <sys/attribs.h>
#include <string.h>
#include "bitbang_i2c.h"
#include "USART.h"
#include "SPI.h"
#include "ili9341.h"
#include "draw.h"
#include "pic32.h"

// Device Config Bits in  DEVCFG1:		
#pragma config FNOSC = SPLL	
#pragma config FSOSCEN = OFF	
//#pragma config FWDTEN = OFF  
#pragma config POSCMOD = OFF	
#pragma config OSCIOFNC = ON	

// Device Config Bits in  DEVCFG2:		
#pragma config FPLLICLK = PLL_FRC	
#pragma config FPLLIDIV = DIV_1	
#pragma config FPLLMULT = MUL_50	
#pragma config FPLLODIV = DIV_2	
#pragma config FPLLRNG = RANGE_5_10_MHZ	
#pragma config FWDTEN = OFF           
#pragma config FDMTEN = OFF  

#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config TRCEN = ON               // Trace Enable (Trace features in the CPU are enabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = ALLOW_PG2       // Debug Mode CPU Access Permission (Allow CPU access to Permission Group 2 permission regions)
#pragma config EJTAGBEN = NORMAL  

#pragma config PGL1WAY = OFF
#pragma config PMDL1WAY = OFF
#pragma config IOL1WAY = OFF


// x1_pin AN12 - RG8
// x2_pin AN9 - RB14
// y1_pin AN13 - RG7
// y2_pin AN10 - RB15
//#define dail1_pin
//#define dail2_pin
#define x1_offset 20350.0
#define x1_min 0.0
#define x1_max 40950.0
#define y1_offset 20400.0
#define y1_min 0.0
#define y1_max 40950.0
#define x2_offset 19900.0
#define x2_min 0.0
#define x2_max 40950.0
#define y2_min 0.0
#define y2_max 40320.0

// ts_xl AN18 - E4
// ts_x2 AN15 - E7
// ts_y1 AN17 - E5
// ts_y2 AN16 - E6
#define ts_x_min 614
#define ts_x_max 3436
#define ts_y_min 390
#define ts_y_max 3116

#define switch1_pin PORTBbits.RB6
#define switch2_pin PORTBbits.RB13

#define LCD_backlight_pin PORTGbits.RG6
#define speaker_pin LATCbits.LATC14

void speaker_tone(float);
void adc_init();
void get_adc_values();
void get_touchscreen();

int analog1_x, analog2_x, analog1_y, analog2_y, c;
int dial1 = 0, dial2 = 0;
int ts_x, ts_y;

char serial_monitor[30][54];

unsigned char receive1;
char rx_buffer[60];
int buffer_counter = 0;

unsigned int rx_time_counter = 0;
bool rx_signal = 0;

char mode = 0;
float pid_p = 0, pid_i = 0, pid_d = 0, altitude_p, altitude_i, altitude_d;//Mode 'A'
unsigned char cursor = 0, arming_counter = 0, GPS_signal = 0, GPS_connected = 0;//Mode 'A'
int arming_time;//Mode 'B'
float pitch, roll, yaw, altitude, latitude, longitude, loop_mode;//Mode 'C'
int acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, compass_x, compass_y, compass_z;//Mode 'D' 

void main(){
    int pre_x1 = 10, pre_y1 = 10, tx1, ty1;
    int pre_dial1 = 3, pre_dial2 = 3;
    int pre_ts_x, pre_ts_y;
    bool rx_signal_flag = 1;
    int i, j;
    unsigned char pre_cursor = 100, pre_mode = 0;
    
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
    ColorLCD_writecommand(0x29);    //Display on  
    FillRect(0, 0, 320, 250, 0xFFFF);//Clear screen   
    
    WriteStr("Dial2", 200, 135, 0x0000);//Dial2    
    FillRect(267, 0, 3, 320, 0xF000);//Throttle divide
    FillRect(25, 135, 87, 87, 0xF000);//Analog indicator box
    FillRect(27, 137, 83, 83, 0xFFFF);//?
    FillRect(114, 143, 64, 24, 0x0000);//Dial 1 box
    FillRect(116, 145, 60, 20, 0xFFFF);//?
    FillRect(134, 145, 3, 20, 0x0000);//?
    FillRect(155, 145, 3, 20, 0x0000);//?
    WriteStr("Dial1", 130, 135, 0x0000);//Dial1
    FillRect(184, 143, 64, 24, 0x0000);//Dial2 box
    FillRect(186, 145, 60, 20, 0xFFFF);//?
    FillRect(204, 145, 3, 20, 0x0000);//?
    FillRect(225, 145, 3, 20, 0x0000);//?
    
    while(1) {
        if(mode == 0 && rx_signal_flag == 1) {
            FillRect(0, 8, 250, 9 * 8, 0xFFFF);
            rx_signal_flag = 0;
        }
        else if(mode != 0) {
            rx_signal_flag = 1;
        }
        if(mode != 'Z') {
            //--------------------------------New mode: clear screen and display new constants for that mode---------------------------
            if(mode != pre_mode){
                FillRect(0, 8, 250, 9 * 8, 0xFFFF);
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
                WriteInt(acc_x, 6, 7*6, 1*8, 0x0000);
                WriteInt(acc_y, 6, 7*6, 2*8, 0x0000);
                WriteInt(acc_z, 6, 7*6, 3*8, 0x0000);
                WriteInt(gyro_x, 6, 8*6, 4*8, 0x0000);
                WriteInt(gyro_y, 6, 8*6, 5*8, 0x0000);
                WriteInt(gyro_z, 6, 8*6, 6*8, 0x0000);
                WriteInt(compass_x, 6, 11*6, 7*8, 0x0000);
                WriteInt(compass_y, 6, 11*6, 8*8, 0x0000);
                WriteInt(compass_z, 6, 11*6, 9*8, 0x0000);
            }
        }

        //------------------------------------------------------Serial Monitor mode-------------------------------------------------

        else {
            if(pre_mode != mode) FillRect(0, 0, 320, 250, 0xFFFF);
            for(i = 0; i < 30; i++){
                if(serial_monitor[i][0] != '\0') WriteStr(serial_monitor[i], 0, i * 8, 0x0000);
            }
            pre_mode = mode;
        }
        
        //--------------------------Display input data: analog sticks, potentiometers and switch values------------------------

        get_touchscreen();
        if(ts_y > 145 && ts_y < (145 + 20)){
            if(ts_x > 116 && ts_x < (116 + 18)) dial1 = 0;
            else if(ts_x > (116 + 21) && ts_x < (116 + 18 + 21)) dial1 = 1;
            else if(ts_x > (116 + 21 * 2) && ts_x < (116 + 18 + 21 * 2)) dial1 = 2;

            else if(ts_x > 186 && ts_x < (186 + 18)) dial2 = 0;
            else if(ts_x > (186 + 21) && ts_x < (186 + 18 + 21)) dial2 = 1;
            else if(ts_x > (186 + 21 * 2) && ts_x < (186 + 18 + 21 * 2)) dial2 = 2;
        }

        tx1 = analog1_x;
        ty1 = analog1_y;
        if(analog2_y){
            if(analog2_y == 31) FillRect(270, 0, 50, 320, 0xFFA0);
            else FillRect(270, ((float)(31 - analog2_y) * 7.75), 50, ((float)(31 - analog2_y) * 7.75), 0xFFA0);
        }
        if(analog2_y != 31) FillRect(270, 0, 50, ((float)(31 - analog2_y) * 7.75), 0xFFFF);
        if(pre_x1 != tx1 || pre_y1 != ty1){
            for(i = 0; i <= 5; i++) DrawCircle(68 + ((float)pre_x1 * 2.4), 178 - ((float)pre_y1 * 2.4), i, 0xFFFF);
            FillRect(27, 178, 83, 1, 0xF000);
            FillRect(68.5, 137, 1, 83, 0xF000);
            for(i = 0; i <= 5; i++) DrawCircle(68 + ((float)tx1 * 2.4), 178 - ((float)ty1 * 2.4), i, 0x0000);
        }
        if(pre_dial1 != dial1){
            if(pre_dial1 >=0 && pre_dial1 < 3) FillRect(116 + (pre_dial1 * 21), 145, 18, 20, 0xFFFF);
            if(dial1 == 0) FillRect(116 + (dial1 * 21), 145, 18, 20, 0xF800);
            else if(dial1 == 1) FillRect(116 + (dial1 * 21), 145, 18, 20, 0x07E0);
            else if(dial1 == 2) FillRect(116 + (dial1 * 21), 145, 18, 20, 0x001F);
        }
        if(pre_dial2 != dial2){
            if(pre_dial2 >=0 && pre_dial2 < 3) FillRect(186 + (pre_dial2 * 21), 145, 18, 20, 0xFFFF);
            if(dial2 == 0) FillRect(186 + (dial2 * 21), 145, 18, 20, 0xF800);
            else if(dial2 == 1) FillRect(186 + (dial2 * 21), 145, 18, 20, 0x07E0);
            else if(dial2 == 2) FillRect(186 + (dial2 * 21), 145, 18, 20, 0x001F);
        }
        WriteFloat(analog2_x, 2, 1, 24 * 6, 23 * 8, 0x0000);
        WriteInt(!switch1_pin, 2, 24 * 6, 24 * 8, 0x0000);
        WriteInt(!switch2_pin, 2, 24 * 6, 25 * 8, 0x0000);

        pre_x1 = tx1;
        pre_y1 = ty1;
        pre_dial1 = dial1;
        pre_dial2 = dial2;
    }
}

void __ISR_AT_VECTOR(_UART1_RX_VECTOR, IPL6SRS) Xbee(void){
    int i, j;
    IFS3bits.U1RXIF = 0; 
    c = 0;
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
                    latitude = 100 * (float)(rx_buffer[30] - 48) + 10 * (float)(rx_buffer[31] - 48) + (float)(rx_buffer[32] - 48) + 0.1 * (float)(rx_buffer[34] - 48) + 0.01 * (float)(rx_buffer[35] - 48) + 0.001 * (float)(rx_buffer[36] - 48) + 0.0001 * (float)(rx_buffer[37] - 48) + 0.00001 * (float)(rx_buffer[38] - 47) + 0.000001 * (float)(rx_buffer[39] - 48) + 0.0000001 * (float)(rx_buffer[40] - 47) + 0.00000001 * (float)(rx_buffer[41] - 48);
                    if(rx_buffer[29] == '-') latitude *= (-1);
                    longitude = 100 * (float)(rx_buffer[43] - 48) + 10 * (float)(rx_buffer[44] - 48) + (float)(rx_buffer[45] - 48) + 0.1 * (float)(rx_buffer[47] - 48) + 0.01 * (float)(rx_buffer[48] - 48) + 0.001 * (float)(rx_buffer[49] - 48) + 0.0001 * (float)(rx_buffer[50] - 48) + 0.00001 * (float)(rx_buffer[51] - 47) + 0.000001 * (float)(rx_buffer[52] - 48) + 0.0000001 * (float)(rx_buffer[53] - 47) + 0.00000001 * (float)(rx_buffer[54] - 48);
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
                }
                else if(mode == 'Z'){
                    buffer_counter = 1;
                    i = 0;
                    j = 0;
                    while(rx_buffer[buffer_counter] != '\0'){
                        if(rx_buffer[buffer_counter++] != '\n'){
                            serial_monitor[j][i++] = rx_buffer[buffer_counter];
                        }
                        else{
                            serial_monitor[j][i] = '\0';
                            i = 0;
                            j++;
                        }
                    }
                    serial_monitor[j++][i] = '\0';
                    for(; j < 30; j++){
                        serial_monitor[j][0] = '\0';
                    }
                    buffer_counter = 0;
                }
            }
        } else {
            rx_buffer[buffer_counter++] = receive1;
        }
        c++;
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
    
    U1TXREG = (analog1_x + 15);
    U1TXREG = (32 + (analog1_y + 15));
    U1TXREG = (64 + (analog2_x + 15));
    U1TXREG = (96 + analog2_y);
    U1TXREG = (128 + !switch1_pin * 2 + !switch2_pin);
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

void get_touchscreen(){
    int i;
    //Set Y resistors as outputs
    TRISEbits.TRISE5 = 0;   //y1
    TRISEbits.TRISE6 = 0;   //y2
    //Set x resistors as inputs
    TRISEbits.TRISE4 = 1;   //x1
    TRISEbits.TRISE7 = 1;   //x2
    //
    LATEbits.LATE5 = 0;     //y1 = -
    LATEbits.LATE6 = 1;     //y2 = +
    
    ANSELEbits.ANSE4 = 1;
    ANSELEbits.ANSE5 = 0;
    
    ts_x = 0;
    for(i = 0; i < 10; i++){
        ADCCON3bits.GSWTRG = 1;
        while(ADCDSTAT1bits.ARDY18 == 0);
        ts_x += ADCDATA18;
    }
    ts_x /= 10;
    
    //Set X resistors as outputs
    TRISEbits.TRISE5 = 1;   //y1
    TRISEbits.TRISE6 = 1;   //y2
    //Set Y resistors as inputs
    TRISEbits.TRISE4 = 0;   //x1
    TRISEbits.TRISE7 = 0;   //x2
    //
    LATEbits.LATE4 = 1;     //x1 = -
    LATEbits.LATE7 = 0;     //x2 = +
    
    ANSELEbits.ANSE4 = 0;
    ANSELEbits.ANSE5 = 1;
    
    ts_y = 0;
    for(i = 0; i < 10; i++){
        ADCCON3bits.GSWTRG = 1;
        while(ADCDSTAT1bits.ARDY17 == 0);
        ts_y += ADCDATA17;
    }
    ts_y /= 10;
    
    ts_x = (320.0 * (float)(ts_x - ts_x_min) / (float)(ts_x_max - ts_x_min));
    ts_y = (240.0 * (float)(ts_y - ts_y_min) / (float)(ts_y_max - ts_y_min));
    if(ts_x > 319) ts_x = 319;
    else if(ts_x < 0) ts_x = 0;   
    if(ts_y > 239) ts_y = 239;
    else if(ts_y < 0) ts_y = 0;
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