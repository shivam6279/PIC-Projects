#define _XTAL_FREQ 48000000
#include <xc.h>
#include <pic18f4550.h>
#include <math.h>
#include "I2C.h"
#include "OLED.h"
#include "USART.h"
#include "10DOF.h"
#include "PWMDriver.h"

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

#define RGBled_red_pin 6
#define RGBled_green_pin 4
#define RGBled_blue_pin 5

#define red_led_pin PORTEbits.RE2
#define blue_led_pin PORTEbits.RE1

#define motor_pin 0
#define aileron_pin 1
#define elevator_pin 2 
#define rudder_pin 3

#define RadToDegrees 57.3
#define PI 3.14159265
#define PI2 6.28318531

#define x_angle_offset 0
#define y_angle_offset 0

typedef struct{
    float p, i, d;
    float sum;
    float angle;
    int output;
} PID;

typedef struct{
    float P[2][2];
    float bias;
    float rate;
    float angle;
} Kalman;

void delay_ms(unsigned int);
void delay_us(unsigned int);
void PIDreset(PID*, float, float, float);
void timer_init();
void init();
void menu(PID*, PID*, PID*, unsigned char*);
void write_RGB_led(unsigned int, unsigned int, unsigned int);
void main();

int safety_counter, x1, y1, x2, y2, loop_counter, led_counter = 0, arming_counter;
unsigned char receive, buttons;

void interrupt ISR(){
    if(RCIF){
        receive = RCREG;
        if(receive / 32 == 0){
            x1 = (receive % 32) - 15;
            safety_counter = 0;
        }
        else if(receive / 32 == 1) y1 = (receive % 32) - 15;
        else if(receive / 32 == 2) x2 = (receive % 32) - 15;
        else if(receive / 32 == 3) y2 = (receive % 32);
        else if(receive / 32 == 4) buttons = (receive % 32);
        if(OERR){
            CREN = 0;
            CREN = 1;
        }
    }
    if(TMR0IF){
        TMR0IF = 0;
        if(safety_counter == 5000){ //If no remote signal has been received in 0.5 second, then set the throttle to 0
            safety_counter = 0;
            x1 = 0;
            y1 = 0;
            x2 = 0;
            y2 = 0;
        }
        else safety_counter++;
        if(arming_counter >= 0){
            if(arming_counter > 0){
                led_counter++;
                if(led_counter < 600){
                    blue_led_pin = 1;
                }
                else if(led_counter > 600){
                    blue_led_pin = 0;
                }
                if(led_counter > 1200) led_counter = 0;
            }
            else{
                led_counter = 0;
                blue_led_pin = 0;
            }
        }
        loop_counter++;
        TMR0 = 184;
    }
}

void main(){
    PID Pitch, Roll, Yaw;
    int throttle;// Motor speeds
    float x_acc_angle, x_offset; //Roll variables
    float y_acc_angle, y_offset; //Pitch variables
    float heading, heading_difference, heading_offset, yaw_offset;//, heading_offsetted_difference; //Yaw variables
    //float x_remote, y_remote, remote_magnitude, remote_angle, remote_angle_difference; // RC variables
    float sin_x, cos_x, sin_y, cos_y;
    float loop_time;
    unsigned char debug, j; //Flags

    init();
    blue_led_pin = 0;
    red_led_pin = 0;
    safety_counter = 0;

    //Initializing all devices: MPU6050, PWM driver and the four ESCs
    
    delay_ms(50);
    LSM303D_init();
    pwm_driver_init(50);
    write_RGB_led(4095, 0, 0);
    delay_ms(100);
    write_pwm(aileron_pin, 307);
    write_pwm(elevator_pin, 307);
    write_pwm(rudder_pin, 307);
    if(y2 > 29){ //Calibrate ESCs
        write_pwm(motor_pin, 410);
        delay_ms(7000);
    }
    write_pwm(motor_pin, 205);
    delay_ms(200);
    init_oled();
    oled_write(0);
    set_xy(0, 30);
    send_str("Property of");
    set_xy(1, 24);
    send_str("Shivam Sharma");
    delay_ms(2000);
    oled_write(0);
    
    while(1){
        PIDreset(&Roll, 2.5, 0.5, 0.9);
        PIDreset(&Pitch, 2.5, 0.5, 0.9);
        PIDreset(&Yaw, 4.5, 0.5, 0.9);
        debug = 0;
        yaw_offset = 0;
        x_offset = -10;
        y_offset = 10;
        oled_write(0);
        set_xy(0, 0);
        send_str("Not Armed");
        timer_init();
        
        //Menu
        menu(&Roll, &Pitch, &Yaw, &debug);
        
        blue_led_pin = 1;
        write_RGB_led(4095, 4095, 0);   
        oled_write(0);
        set_xy(2, 0);
        send_str("Arming");
        for(j = 0, heading_offset = 0; j < 100; j++){
            get_acc();
            get_compass();
            x_acc_angle = -atan2(acc.y, sqrt(acc.z * acc.z + acc.x * acc.x)) - x_angle_offset;
            y_acc_angle = -atan2(acc.x, sqrt(acc.z * acc.z + acc.y * acc.y)) - y_angle_offset;
            sin_x = sin(x_acc_angle);
            cos_x = cos(x_acc_angle);
            sin_y = sin(y_acc_angle);
            cos_y = cos(y_acc_angle);
            //heading_offset += get_compensated_heading(sin_x, cos_x, sin_y, cos_y);
            heading_offset += atan2(compass.y, compass.x);
            delay_ms(7);
        }
        heading_offset /= 100;
        oled_write(0);
        set_xy(0, 0);
        send_str("Armed ");  
        write_RGB_led(0, 4095, 0);  
        
        //Main Loop
        while(1){
            loop_counter = 0;
            
            //----------------------------------------------------Remote control receiver data-----------------------------------------------------------------------
            
            if(buttons % 2 == 1) break;// Arm/Disarm switch
            if((buttons / 2) % 2 == 1){// If killed:
                write_pwm(motor_pin, 2040);    // Stop all motors
                oled_write(0);
                set_xy(0, 0);
                send_str("Armed ");
                set_xy(2, 0);
                send_str("Killed");
                while((buttons / 2) % 2 == 1){
                    if(buttons % 2 == 1) break; // Wait to be Un-killed
                }
                if(buttons % 2 == 1) break;
                set_xy(2, 0);
                send_str("      ");
            }
            throttle = y2 * 66; // Range throttle from 0 - 2046
            yaw_offset += x2 / 12 / RadToDegrees;
            if(yaw_offset < (-PI)) yaw_offset += PI2;
            else if(yaw_offset > PI) yaw_offset -= PI2;
            
            //--------------------------------------------------------IMU data acquisition---------------------------------------------------------------------------
            
            get_acc();
            //get_gyro();
            get_compass();            
            
            //------------------------------------------Calculating angles: Pitch(x-axis) and Roll(y-axis)-----------------------------------------------------------
            
            x_acc_angle = (-atan2(acc.y, sqrt(acc.z * acc.z + acc.x * acc.x)) - x_angle_offset);
            y_acc_angle = (-atan2(acc.x, sqrt(acc.z * acc.z + acc.y * acc.y)) - y_angle_offset);
            //Calculate sin and cos for both angles; needed to rotate vectors for compass and gyro tilt-compensation
            sin_x = sin(x_acc_angle);
            cos_x = cos(x_acc_angle);
            sin_y = sin(y_acc_angle);
            cos_y = cos(y_acc_angle);
            
            //-----------------------------------------------------------Heading calculation-------------------------------------------------------------------------
            
            //heading = get_compensated_heading(sin_x, cos_x, sin_y, cos_y);
            heading = atan2(compass.y, compass.x);
            heading_difference = heading - heading_offset;
            if(heading_difference < (-PI)) heading_difference += PI2;
            else if(heading_difference > PI) heading_difference -= PI2;
            //heading_offsetted_difference = heading_difference - yaw_offset;
            //if(heading_offsetted_difference < (-PI)) heading_offsetted_difference += PI2;
            //else if(heading_offsetted_difference > PI) heading_offsetted_difference -= PI2;
            
            //------------------------------------------------Converting Remote data to a 2-D vector------------------------------------------------------------------
            
            /*
            remote_magnitude = sqrt(pow(x1, 2) + pow(y1, 2)); //Magnitude of remote's roll and pitch
            remote_angle = atan2(x1, y1);                     //Angle with respect to pilot/starting position
            if(x1 == 0 && y1 == 0) remote_angle = 0;
            remote_angle_difference = remote_angle - heading_difference;//Remote's angle with respect to quad's current direction
            if(remote_angle_difference < (-PI)) remote_angle_difference += PI2;
            else if(remote_angle_difference > PI) remote_angle_difference -= PI2;
            y_remote = 2 * remote_magnitude * cos(remote_angle_difference);
            x_remote = 2 * remote_magnitude * sin(remote_angle_difference);
            */
            
            //-----------------------------------------------Filters: Fusion, complimentary, Kalman-------------------------------------------------------------------
            
            /*
            Roll.angle = (0.97 * (Roll.angle + gyro.x * loop_time) + 0.03 * (x_acc_angle * RadToDegrees));    //Complimentary Filter
            Pitch.angle = (0.97 * (Pitch.angle + gyro.y * loop_time) + 0.03 * (-y_acc_angle * RadToDegrees));
            //Roll.angle = KalmanGetAngle(&KalmanRoll, RadToDegrees * x_acc_angle - x_remote, gyro.x, loop_time);         //Kalman Filter
            //Pitch.angle = KalmanGetAngle(&KalmanPitch, -RadToDegrees * y_acc_angle - y_remote, gyro.y, loop_time);
            //Yaw.angle = (0.97 * (Yaw.angle - (-gyro.x * sin_x + gyro.y * sin_x * sin_y + gyro.z * cos_x * cos_y) * loop_time) + 0.03 * (heading_difference * RadToDegrees));
            Yaw.angle -= gyro.z * loop_time;
            */
            
            //-----------------------------------------------------------------PID-------------------------------------------------------------------------------------
            
            /*
            Roll.sum += (Roll.angle - x_remote) * loop_time * 3;
            Pitch.sum += (Pitch.angle - y_remote) * loop_time * 3;
            Yaw.sum += (Yaw.angle - yaw_offset * RadToDegrees) / 20;
            Roll.output = (int)(float)(Roll.p * (Roll.angle - x_remote) + Roll.i * Roll.sum + Roll.d * gyro.x); //Roll PID
            Pitch.output = (int)(float)(Pitch.p * (Pitch.angle - y_remote) + Pitch.i * Pitch.sum + Pitch.d * gyro.y); //Pitch PID
            Yaw.output = (int)(float)(Yaw.p * (Yaw.angle - yaw_offset * RadToDegrees) + Yaw.i * Yaw.sum - Yaw.d * gyro.z); //Yaw PID
            */
            
            //-------------------------------------------------------------Motor Output--------------------------------------------------------------------------------
            
            
            //---------------------------------------------------------------Display-----------------------------------------------------------------------------------
            
            if(debug == 1){
                set_xy(2, 0);
                write_int(loop_counter, 3);
                set_xy(3, 0);
                write_float(x_acc_angle * RadToDegrees, 3, 3);
                set_xy(4, 0);
                write_float(y_acc_angle * RadToDegrees, 3, 3);
                set_xy(5, 0);
                write_float(heading_difference * RadToDegrees, 3, 3);
            }
            
            //-----------------------------------------------------------Output to ESC's-------------------------------------------------------------------------------
            
            else{
                write_pwm(motor_pin, 205);
                write_pwm(aileron_pin, 307 - (int)(float)(x_acc_angle * RadToDegrees * 2.4));
                write_pwm(elevator_pin, 307 + (int)(float)(y_acc_angle * RadToDegrees * 2.4));
                write_pwm(rudder_pin, 307 + (int)(float)(heading * RadToDegrees * 2.4));
            }
            
            //---------------------------------------------------------------------------------------------------------------------------------------------------------
            
            loop_time = (float)loop_counter / 10000; // Loop time in milliseconds: 20-30ms
        }
        //After killing and breaking from the main loop, stop all motors
        TMR0IE = 0;
        blue_led_pin = 0;
        write_pwm(motor_pin, 205);
        write_pwm(aileron_pin, 307);
        write_pwm(elevator_pin, 307);
        write_pwm(rudder_pin, 307);
        write_RGB_led(4095, 0, 0);
        oled_write(0);
        set_xy(0, 0);
        send_str("Disarmed");
        set_xy(2, 0);
        send_str("x offset: ");
        write_float(x_offset, 2, 1);
        set_xy(3, 0);
        send_str("y_offset: ");
        write_float(y_offset, 2, 1);
        set_xy(4, 0);
        send_str("P: ");
        write_float(Roll.p, 2, 1);
        set_xy(5, 0);
        send_str("I: ");
        write_float(Roll.i, 2, 1);
        set_xy(6, 0);
        send_str("D: ");
        write_float(Roll.d, 2, 1);
        set_xy(7, 0);
        send_str("B: ");
        write_int(buffer_size, 1);
        while(buttons % 2 == 1);
    }
}

void menu(PID *x, PID *y, PID *z, unsigned char *d){
    unsigned char flag_menu = 1, i;
    signed char cursor = 0, buffer_temp = 0;
    unsigned int r, g, b, led_counter = 0;
    arming_counter = 0;
    while(arming_counter < 20){
        //--------LED stuff---------
        if(led_counter < 4096){
            r = 4095;
            g = led_counter;
            b = 0;
        }
        else if(led_counter < 8192){
            r = 4095 - (led_counter - 4096);
            g = 4095;
            b = 0;
        }
        else if(led_counter < 12288){
            r = 0;
            g = 4095;
            b = led_counter - 8192;
        }
        else if(led_counter < 16384){
            r = 0;
            g = 4095 - (led_counter - 12288);
            b = 4095;
        }
        else if(led_counter < 20480){
            r = led_counter - 16384;
            g = 0;
            b = 4095;
        }
        else{
            r = 4095;
            g = 0;
            b = 4095 - (led_counter - 20480);
        }
        led_counter += 150;
        if(led_counter >= 24576){
            led_counter = 0;
        }
        write_RGB_led(r, g, b);
        //-----------------------------
        set_xy(0, 69);
        send_line(255);
        for(i = 1; i < 20; i++){
            if(i <= arming_counter){
                send_line(255);
                send_line(255);
                send_line(255);
            }
            else{            
                send_line(129);
                send_line(129);
                send_line(129);
            }
        }
        send_line(255);
        set_xy(1, 0);
        if(PORTDbits.RD0 == 0) send_str("No signal      ");
        else send_str("Signal received");
        if(y1 > 12 && flag_menu == 0){
            cursor--;
            flag_menu = 1;
        }
        else if(y1 < (-12) && flag_menu == 0){
            cursor++;
            flag_menu = 1;
        }
        else if(y1 > (-12) && y1 < 12) flag_menu = 0;
        if(cursor > 3) cursor = 3;
        else if(cursor < 0) cursor = 0;
        set_xy(2, 0);
        if(cursor == 0){
            send_char('>');
            if(x1 > 12){
                x->p += 0.1;
                y->p += 0.1;
                z->p += 0.1;
            }
            else if(x1 < (-12)){
                x->p -= 0.1;
                y->p -= 0.1;
                z->p - = 0.1;
            }
        }
        else send_char(' ');
        send_str("P = ");
        write_float(x->p, 2, 1);
        set_xy(3, 0);
        if(cursor == 1){
            send_char('>');
            if(x1 > 12){
                x->i += 0.1;
                y->i += 0.1;
                z->i += 0.1;
            }
            else if(x1 < (-12)){
                x->i -= 0.1;
                y->i -= 0.1;
                z->i - = 0.1;
            }
        }
        else send_char(' ');
        send_str("I = ");
        write_float(x->i, 2, 1);
        set_xy(4, 0);
        if(cursor == 2){
            send_char('>');
            if(x1 > 12){
                x->d += 0.1;
                y->d += 0.1;
                z->d += 0.1;
            }
            else if(x1 < (-12)){
                x->d -= 0.1;
                y->d -= 0.1;
                z->d - = 0.1;
            }
        }
        else send_char(' ');
        send_str("D = ");
        write_float(x->d, 2, 1);
        set_xy(5, 0);
        if(cursor == 3){
            send_char('>');
            if(x1 > 12) *d = 1;
            else if(x1 < (-12)) *d = 0;
        }
        else send_char(' ');
        send_str("Debug: ");
        if(*d == 0) send_str("Off");
        else send_str("On ");
        send_char(' ');
        delay_ms(25);
        if((buttons == 2) && y2 < 2 && x2 > 13) arming_counter++;
        else arming_counter = 0;
    }
    buffer_size = buffer_temp;
    arming_counter = -1;
}

void write_RGB_led(unsigned int r, unsigned int g, unsigned int b){
    write_pwm(RGBled_red_pin, (4095 - r));
    write_pwm(RGBled_green_pin, (4095 - g));
    write_pwm(RGBled_blue_pin, (4095 - b));
}

void delay_ms(unsigned int x){
    unsigned int i;
    for(i = 0; i < x; i++) __delay_ms(1);
}

void delay_us(unsigned int x){
    unsigned int i;
    for(i = 0; i < x; i++) __delay_us(1);
}

void PIDreset(PID *x, float kp, float ki, float kd){
    x->p = kp;
    x->i = ki;
    x->d = kd;
    x->sum = 0;
    x->output = 0;
    x->angle = 0;
}

void timer_init(){
    T0CS = 0;
    T0SE = 0;
    PSA = 0;
    T0PS2 = 0;
    T0PS1 = 1;
    T0PS0 = 1;
    IPEN = 0;
    GIE = 1;
    TMR0 = 181;
    TMR0IE = 1;
    TMR0ON = 1;
}

void init(){
    TRISA = 1;
    TRISB = 0;
    TRISC = 0;
    TRISD = 1;
    TRISE = 0;
    ADCON0 = 0;
    ADCON1 = 15;
    USART_init(9600, 1);
    i2c_init();
    receive = 0;
    //receive_buttons = 0;
    x1 = 0;
    y1 = 0;
    x2 = 0;
    y2 = 0;
}