#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include <sys/attribs.h>
#include <string.h>
#include "bitbang_i2c.h"
#include "USART.h"
#include "SPI.h"
#include "ili9341.h"

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

#define LCD_backlight_pin PORTGbits.RG6
#define touchscreen_xl LATEbits.RE4//AN18
#define touchscreen_yu LATEbits.RE5//AN17
#define touchscreen_xr LATEbits.RE7//AN15
#define touchscreen_yd LATEbits.RE6//AN16

#define speaker_pin LATCbits.LATC14

// x1_pin AN12 - RG8
// x2_pin AN9 - RB14
// y1_pin AN13 - RG7
// y2_pin AN10 - RB15
//#define dail1_pin
//#define dail2_pin
#define switch1_pin PORTBbits.RB6
#define switch2_pin PORTBbits.RB13

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

void delay_ms(unsigned int);
void timer2_init();
void timer3_init();
void timer4_init(float);
void adc_init();
void get_adc_values();
void init();

unsigned long int delay_counter = 0;

int analog1_x, analog2_x, analog1_y, analog2_y, dial1, dial2, c;

char serial_monitor[30][54];

unsigned char receive1;
char rx_buffer[60];
int buffer_counter = 0;

char mode;
float pid_p = 0, pid_i = 0, pid_d = 0, altitude_p, altitude_i, altitude_d;//Mode 'A'
unsigned char cursor = 0, arming_counter = 0, GPS_signal = 0;//Mode 'A'
int arming_time;//Mode 'B'
float pitch, roll, yaw, altitude, latitude, longitude, loop_mode;//Mode 'C'
int acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, compass_x, compass_y, compass_z;//Mode 'D' 

void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4SRS) delay_timer(void){
    IFS0bits.T2IF = 0;
    delay_counter++;
}

void __ISR_AT_VECTOR(_UART1_RX_VECTOR, IPL6SRS) Xbee(void){
    int i, j;
    IFS3bits.U1RXIF = 0; 
    c = 0;
    do{
        receive1 = U1RXREG & 0xFF;
        if(receive1 == '\r'){
            rx_buffer[buffer_counter] = '\0';
            buffer_counter = 0;
            if(rx_buffer[0] > 64 && rx_buffer[0] < 91){
                mode = rx_buffer[0];
                if(mode == 'A'){
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
                    arming_counter = 10 * (rx_buffer[39] - 48) + (rx_buffer[40] - 48);
                }
                else if(mode == 'B'){
                    arming_time = (100 * (rx_buffer[1] - 48) + 10 * (rx_buffer[2] - 48) + (rx_buffer[3] - 48));
                }
                else if(mode == 'C'){
                    pitch = 100 * (float)(rx_buffer[2] - 48) + 10 * (float)(rx_buffer[3] - 48) + (float)(rx_buffer[4] - 48) + 0.1 * (float)(rx_buffer[6] - 48) + 0.01 * (float)(rx_buffer[7] - 48);
                    if(rx_buffer[1] == '-') pitch *= (-1);
                    roll = 100 * (float)(rx_buffer[9] - 48) + 10 * (float)(rx_buffer[10] - 48) + (float)(rx_buffer[11] - 48) + 0.1 * (float)(rx_buffer[13] - 48) + 0.01 * (float)(rx_buffer[14] - 48);
                    if(rx_buffer[8] == '-') roll *= (-1);
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
        }
        else{
            rx_buffer[buffer_counter++] = receive1;
        }
        c++;
    }while(U1STAbits.URXDA);
    U1STAbits.OERR = 0;
    IFS3bits.U1RXIF = 0; 
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

void DrawCircle(int x0, int y0, int radius, unsigned int color){
	int x = 0, y = radius;
	int dp = 1 - radius;
	do{
		if (dp < 0) dp = dp + 2 * (++x) + 3;
		else dp = dp + 2 * (++x) - 2 * (--y) + 5;

		ColorLCD_draw_pixel(x0 + x, y0 + y, color);     //For the 8 octants
		ColorLCD_draw_pixel(x0 - x, y0 + y, color);
		ColorLCD_draw_pixel(x0 + x, y0 - y, color);
		ColorLCD_draw_pixel(x0 - x, y0 - y, color);
		ColorLCD_draw_pixel(x0 + y, y0 + x, color);
		ColorLCD_draw_pixel(x0 - y, y0 + x, color);
		ColorLCD_draw_pixel(x0 + y, y0 - x, color);
		ColorLCD_draw_pixel(x0 - y, y0 - x, color);

	}while(x < y);
    ColorLCD_draw_pixel(x0 + radius, y0, color);
    ColorLCD_draw_pixel(x0 - radius, y0, color);
    ColorLCD_draw_pixel(x0, y0 + radius, color);
    ColorLCD_draw_pixel(x0, y0 - radius, color);
}

void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL7SRS) speaker_timer(void){
    IFS0bits.T4IF = 0;
    LATCINV = 0x4000;
    //speaker_pin = !speaker_pin;
}

void main(){
    int pre_x1 = 10, pre_y1 = 10, pre_dial1 = 3, pre_dial2 = 3, tx1, ty1;
    int i, j;
    unsigned char pre_cursor = 100, pre_mode = 0;
    
    init();
    adc_init();
    timer2_init();
    timer3_init();
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
    ColorLCD_fillRect(0, 0, 320, 250, 0xFFFF);//Clear screen    
    ColorLCD_fillRect(267, 0, 3, 320, 0xF000);//Throttle divide
    ColorLCD_fillRect(25, 135, 87, 87, 0xF000);//Analog indicator box
    ColorLCD_fillRect(27, 137, 83, 83, 0xFFFF);//?
    ColorLCD_fillRect(114, 143, 64, 24, 0x0000);//Dial 1 box
    ColorLCD_fillRect(116, 145, 60, 20, 0xFFFF);//?
    ColorLCD_fillRect(134, 145, 3, 20, 0x0000);//?
    ColorLCD_fillRect(155, 145, 3, 20, 0x0000);//?
    ColorLCD_write_str("Dial1", 130, 135, 0x0000);//Dial1
    ColorLCD_fillRect(184, 143, 64, 24, 0x0000);//Dial2 box
    ColorLCD_fillRect(186, 145, 60, 20, 0xFFFF);//?
    ColorLCD_fillRect(204, 145, 3, 20, 0x0000);//?
    ColorLCD_fillRect(225, 145, 3, 20, 0x0000);//?
    ColorLCD_write_str("Dial2", 200, 135, 0x0000);//Dial2
    
    while(1){
        if(mode != 'Z'){
            //--------------------------------New mode: clear screen and display new constants for that mode---------------------------
            if(mode != pre_mode){
                ColorLCD_fillRect(0, 8, 250, 8 * 7, 0xFFFF);
                if(mode == 'A'){
                    ColorLCD_write_str("  P:", 0, 1 * 8, 0x0000);
                    ColorLCD_write_str("  I:", 0, 2 * 8, 0x0000);
                    ColorLCD_write_str("  D:", 0, 3 * 8, 0x0000);
                    ColorLCD_write_str("  Altitude-p:", 0, 4 * 8, 0x0000);
                    ColorLCD_write_str("  Altitude-i:", 0, 5 * 8, 0x0000);
                    ColorLCD_write_str("  Altitude-d:", 0, 6 * 8, 0x0000);
                    ColorLCD_write_str("  GPS signal:", 0, 7 * 8, 0x0000);
                    ColorLCD_fillRect(80, 9, 62, 10, 0xF000);
                    ColorLCD_fillRect(81, 10, 60, 8, 0xFFFF);
                }
                else if(mode == 'B'){
                    ColorLCD_write_str(" Arming:", 0, 1 * 8, 0x0000);
                    ColorLCD_fillRect(1, 19, 102, 17, 0xF000);
                    ColorLCD_fillRect(2, 20, 100, 15, 0xFFFF);
                }
                else if(mode == 'C'){
                    ColorLCD_write_str(" Mode:", 0, 2 * 8, 0x0000);
                    ColorLCD_write_str(" Pitch:", 0, 2 * 8, 0x0000);
                    ColorLCD_write_str(" Roll:", 0, 3 * 8, 0x0000);
                    ColorLCD_write_str(" Yaw:", 0, 4 * 8, 0x0000);
                    ColorLCD_write_str(" Altitude:", 0, 5 * 8, 0x0000);
                    ColorLCD_write_str(" Latitude:", 0, 6 * 8, 0x0000);
                    ColorLCD_write_str(" Longitude:", 0, 7 * 8, 0x0000);
                }
            }
            pre_mode = mode;
            
            //--------------------------------------------Display variables for the current mode---------------------------------------

            if(mode == 'A'){
                if(cursor != pre_cursor){
                    ColorLCD_write_str(" ", 4, (pre_cursor + 1) * 8, 0x0000);
                    ColorLCD_write_str(">", 4, (cursor + 1) * 8, 0x0000);
                }
                pre_cursor = cursor;
                ColorLCD_write_float(pid_p, 2, 2, 5 * 6, 1 * 8, 0x0000);
                ColorLCD_write_float(pid_i, 2, 2, 5 * 6, 2 * 8, 0x0000);
                ColorLCD_write_float(pid_d, 2, 2, 5 * 6, 3 * 8, 0x0000);
                ColorLCD_write_float(altitude_p, 3, 1, 14 * 6, 4 * 8, 0x0000);
                ColorLCD_write_float(altitude_i, 2, 2, 14 * 6, 5 * 8, 0x0000);
                ColorLCD_write_float(altitude_d, 3, 1, 14 * 6, 6 * 8, 0x0000);
                if(GPS_signal) ColorLCD_write_str("Yes", 14 * 6, 7 * 8, 0x0000);
                else ColorLCD_write_str("No ", 14 * 6, 7 * 8, 0x0000);
                if(arming_counter) ColorLCD_fillRect(81, 10, arming_counter * 3, 8, 0xFFA0);
                else ColorLCD_fillRect(81, 10, 60, 8, 0xFFFF);
            }

            if(mode == 'B'){
                ColorLCD_fillRect(2, 20, (arming_time / 10) + 1, 15, 0xFFA0);
            }

            if(mode == 'C'){
                if(loop_mode == 'S') ColorLCD_write_str("RC      ", 7 * 6, 1 * 8, 0xF80F);
                else if(loop_mode == 'A')ColorLCD_write_str("Alt-hold", 7 * 6, 1 * 8, 0xF80F);
                else if(loop_mode == 'P')ColorLCD_write_str("Pos-hold", 7 * 6, 1 * 8, 0xF80F);
                else if(loop_mode == 'K')ColorLCD_write_str("Killed  ", 7 * 6, 1 * 8, 0xF80F);
                ColorLCD_write_float(pitch, 3, 2, 8 * 6, 2 * 8, 0x0000);
                ColorLCD_write_float(roll, 3, 2, 7 * 6, 3 * 8, 0x0000);
                ColorLCD_write_float(yaw, 3, 2, 6 * 6, 4 * 8, 0x0000);
                ColorLCD_write_float(altitude, 3, 2, 11 * 6, 5 * 8, 0x0000);
                ColorLCD_write_float(latitude, 2, 6, 11 * 6, 6 * 8, 0x0000);
                ColorLCD_write_float(longitude, 2, 6, 12 * 6, 7 * 8, 0x0000);
            }
            
            if(mode == 'D'){
                ColorLCD_write_str("Calibration", 0, 0, 0xF80F);
                ColorLCD_write_str("Acc x:", 0, 1*8, 0xF80F);
                ColorLCD_write_str("Acc y:", 0, 2*8, 0xF80F);
                ColorLCD_write_str("Acc z:", 0, 3*8, 0xF80F);
                ColorLCD_write_str("Gyro x:", 0, 4*8, 0xF80F);
                ColorLCD_write_str("Gyro y:", 0, 5*8, 0xF80F);
                ColorLCD_write_str("Gyro z:", 0, 6*8, 0xF80F);
                ColorLCD_write_str("Compass x:", 0, 7*8, 0xF80F);
                ColorLCD_write_str("Compass y:", 0, 8*8, 0xF80F);
                ColorLCD_write_str("Compass z:", 0, 9*8, 0xF80F);
                ColorLCD_write_int(acc_x, 6, 7*6, 1*8, 0x0000);
                ColorLCD_write_int(acc_y, 6, 7*6, 2*8, 0x0000);
                ColorLCD_write_int(acc_z, 6, 7*6, 3*8, 0x0000);
                ColorLCD_write_int(gyro_x, 6, 8*6, 4*8, 0x0000);
                ColorLCD_write_int(gyro_y, 6, 8*6, 5*8, 0x0000);
                ColorLCD_write_int(gyro_z, 6, 8*6, 6*8, 0x0000);
                ColorLCD_write_int(compass_x, 6, 11*6, 7*8, 0x0000);
                ColorLCD_write_int(compass_y, 6, 11*6, 8*8, 0x0000);
                ColorLCD_write_int(compass_z, 6, 11*6, 9*8, 0x0000);
            }

            //--------------------------Display input data: analog sticks, potentiometers and switch values------------------------
            
            tx1 = analog1_x;
            ty1 = analog1_y;
            if(analog2_y){
                if(analog2_y == 31) ColorLCD_fillRect(270, 0, 50, 320, 0xFFA0);
                else ColorLCD_fillRect(270, ((float)(31 - analog2_y) * 7.75), 50, ((float)(31 - analog2_y) * 7.75), 0xFFA0);
            }
            if(analog2_y != 31) ColorLCD_fillRect(270, 0, 50, ((float)(31 - analog2_y) * 7.75), 0xFFFF);
            if(pre_x1 != tx1 || pre_y1 != ty1){
                for(i = 0; i <= 5; i++) DrawCircle(68 + ((float)pre_x1 * 2.4), 178 - ((float)pre_y1 * 2.4), i, 0xFFFF);
                ColorLCD_fillRect(27, 178, 83, 1, 0xF000);
                ColorLCD_fillRect(68.5, 137, 1, 83, 0xF000);
                for(i = 0; i <= 5; i++) DrawCircle(68 + ((float)tx1 * 2.4), 178 - ((float)ty1 * 2.4), i, 0x0000);
            }
            if(pre_dial1 != dial1){
                if(pre_dial1 >=0 && pre_dial1 < 3) ColorLCD_fillRect(116 + (pre_dial1 * 21), 145, 18, 20, 0xFFFF);
                if(dial1 == 0) ColorLCD_fillRect(116 + (dial1 * 21), 145, 18, 20, 0xF800);
                else if(dial1 == 1) ColorLCD_fillRect(116 + (dial1 * 21), 145, 18, 20, 0x07E0);
                else if(dial1 == 2) ColorLCD_fillRect(116 + (dial1 * 21), 145, 18, 20, 0x001F);
            }
            if(pre_dial2 != dial2){
                if(pre_dial2 >=0 && pre_dial2 < 3) ColorLCD_fillRect(186 + (pre_dial2 * 21), 145, 18, 20, 0xFFFF);
                if(dial2 == 0) ColorLCD_fillRect(186 + (dial2 * 21), 145, 18, 20, 0xF800);
                else if(dial2 == 1) ColorLCD_fillRect(186 + (dial2 * 21), 145, 18, 20, 0x07E0);
                else if(dial2 == 2) ColorLCD_fillRect(186 + (dial2 * 21), 145, 18, 20, 0x001F);
            }
            ColorLCD_write_float(analog2_x, 2, 1, 24 * 6, 23 * 8, 0x0000);
            ColorLCD_write_int(!switch1_pin, 2, 24 * 6, 24 * 8, 0x0000);
            ColorLCD_write_int(!switch2_pin, 2, 24 * 6, 25 * 8, 0x0000);
            
            pre_x1 = tx1;
            pre_y1 = ty1;
            pre_dial1 = dial1;
            pre_dial2 = dial2;
        }
        
        //------------------------------------------------------Serial Monitor mode-------------------------------------------------
        
        else{
            if(pre_mode != mode) ColorLCD_fillRect(0, 0, 320, 250, 0xFFFF);
            for(i = 0; i < 30; i++){
                if(serial_monitor[i][0] != '\0') ColorLCD_write_str(serial_monitor[i], 0, i * 8, 0x0000);
            }
            pre_mode = mode;
        }
    }
}

void adc_init(){
    ADCCON1bits.ON = 0; 
    
    ADCCON1 = 0;
    ADCCON1bits.STRGSRC = 1;
    
    ADCCON2bits.SAMC = 1023;
    ADCCON2bits.ADCDIV = 4;
    
    ADCCON3 = 0;
    ADCCON3bits.ADCSEL = 0;
    ADCCON3bits.CONCLKDIV = 1;
    ADCCON3bits.VREFSEL = 0;
    
    ADCIMCON1 = 0;
    ADCIMCON2 = 0;
    ADCIMCON3 = 0;
    
    ADCGIRQEN1 = 0;
    ADCGIRQEN2 = 0;
    
    ADCCMPCON1 = 0;                    
    ADCCMPCON2 = 0;
    ADCCMPCON3 = 0;
    ADCCMPCON4 = 0;
    ADCCMPCON5 = 0;
    ADCCMPCON6 = 0;
    
    ADCFLTR1 = 0;                     
    ADCFLTR2 = 0;
    ADCFLTR3 = 0;
    ADCFLTR4 = 0;
    ADCFLTR5 = 0;
    ADCFLTR6 = 0;
    
    ADCEIEN1 = 0; 
    ADCEIEN2 = 0;
    
    ADCTRGSNS = 0;
    
    ADCTRG1 = 0;                      
    ADCTRG2 = 0;                   
    ADCTRG3 = 0;    
    
    ADCTRG3bits.TRGSRC9 = 1;
    ADCTRG3bits.TRGSRC10 = 1;
    ADCTRG2bits.TRGSRC7 = 1;
    
    ADCCSS1 = 0;
    ADCCSS2 = 0;
    ADCCSS1bits.CSS9 = 1;
    ADCCSS1bits.CSS10 = 1;
    ADCCSS1bits.CSS12 = 1;
    ADCCSS1bits.CSS13 = 1;
    ADCCSS1bits.CSS15 = 1;
    ADCCSS1bits.CSS16 = 1;
    ADCCSS1bits.CSS17 = 1;
    ADCCSS1bits.CSS18 = 1;
    
    ADCANCONbits.WKUPCLKCNT = 5;
    
    ADCCON1bits.ON = 1; 
    
    while(!ADCCON2bits.BGVRRDY);
    while(ADCCON2bits.REFFLT); 
    
    ADCANCONbits.ANEN7 = 1;
    ADCCON3bits.DIGEN7 = 1;
}

void get_adc_values(){
    int i;
    analog1_x = 0;
    analog1_y = 0;
    analog2_x = 0;
    analog2_y = 0;
    dial1 = 0;
    dial2 = 0;
    for(i = 0; i < 10; i++){
        ADCCON3bits.GSWTRG = 1;
        while(ADCDSTAT1bits.ARDY9 == 0 || ADCDSTAT1bits.ARDY10 == 0 || ADCDSTAT1bits.ARDY12 == 0 || ADCDSTAT1bits.ARDY13 == 0);
        analog1_x += ADCDATA12;
        analog2_x += ADCDATA9;
        analog1_y += ADCDATA13;
        analog2_y += ADCDATA10;
        //dial1 += ADCDATA5;
        //dial2 += ADCDATA6;
    }
    analog1_x = 31.5 * (analog1_x - x1_offset) / (x1_max - x1_min);
    analog1_y = 31.5 * (analog1_y - y1_offset) / (y1_max - y1_min);
    analog2_x = 31.5 * (analog2_x - x2_offset) / (x2_max - x2_min);
    analog2_y = 31.5 * (analog2_y - y2_min) / (y2_max - y2_min);
    //dial1 = (int)(float)(sqrt((float)dial1) / 250.0);
    //dial2 = (int)(float)(sqrt((float)(dial2 - 400)) / 250.0);
    if(analog1_x > 15) analog1_x = 15;
    else if(analog1_x < (-15)) analog1_x = (-15);
    if(analog1_y > 15) analog1_y = 15;
    else if(analog1_y < (-15)) analog1_y = (-15);
    if(analog2_x > 15) analog2_x = 15;
    else if(analog2_x < (-15)) analog2_x = (-15);
    /*if(analog2_y > 31) analog2_y = 31;
    else if(analog2_y < 0) analog2_y = 0;
    if(dial1 > 2) dial1 = 2;
    else if(dial1 < 0) dial1 = 0;
    if(dial2 > 2) dial2 = 2;
    else if(dial2 < 0) dial2 = 0;*/
}

void init(){
    //IO pins
    TRISB = 0xE040;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0xF0;
    TRISF = 0;
    TRISG = 0x0180;
    ANSELB = 0xC000;
    ANSELE = 0xF0;
    ANSELG = 0x0180;
    
    PRECONbits.PREFEN = 3;
    PRECONbits.PFMWS = 2;
    SYSKEY = 0xAA996655;//Unlocking
    SYSKEY = 0x556699AA;//Sequence
    OSCCONbits.FRCDIV = 0;
    OSCCONbits.COSC = 1;
    OSCTUNbits.TUN = 0;
    //SYSKEY = 0x33333333;//Locking sequence
    
    PRISS = 0x76543210;
    INTCONbits.MVEC = 1;
    
    PB2DIVbits.ON = 1;
    PB2DIVbits.PBDIV = 1;//PBCLK2 at 100mhz
    
    PB3DIVbits.ON = 1;
    PB3DIVbits.PBDIV = 1;//PBCLK3 at 100mhz
    
    __asm__("ei");//Enable interrupts
}

void delay_ms(unsigned int x){
    delay_counter = 0;
    T2CONbits.TON = 1;
    while(delay_counter < x);
    T2CONbits.TON = 0;
}

void timer2_init(){
    T2CONbits.TON = 0;
    T2CONbits.TCKPS = 5;//1Khz
    PR2 = 3125;
    TMR2 = 0;
    IPC2bits.T2IP = 4;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    T2CONbits.TON = 0;
}

void timer3_init(){
    T3CONbits.TON = 0;
    T3CONbits.TCKPS = 7;//50hz
    PR3 = 7812;
    TMR3 = 0;
    IPC3bits.T3IP = 3;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    T3CONbits.TON = 1;
}

void timer4_init(float frequency){
    float t = 100000000.0 / frequency; unsigned char pre = 0;
    while(t > 65535){ t /= 2.0; pre++; }
    t = (int)t;
    while((int)t % 2 == 0 && pre < 8){ t /= 2.0; pre++; }
    if(pre == 7){ t *= 2.0; pre--; }
    if(pre == 8) pre = 7;
    T4CONbits.ON = 0;
    T4CONbits.TCKPS = pre;
    PR4 = (int)t - 1;
    TMR4 = 0;
    IPC4bits.T4IP = 7;
    IFS0bits.T4IF = 0;
    IEC0bits.T4IE = 1;
    T4CONbits.TON = 1;
}