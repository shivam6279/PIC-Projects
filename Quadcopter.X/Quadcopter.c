#define big

#ifdef micro
#warning "--------------------------------Building for Micro quad!--------------------------------"
#endif
#ifdef mini
#warning "--------------------------------Building for Mini quad!--------------------------------"
#endif
#ifdef big
#warning "--------------------------------Building for Big quad!--------------------------------"
#endif

#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include <sys/attribs.h>  
#include "bitbang_i2c.h"
#include "PWMDriver.h"
#include "10DOF.h"
#include "USART.h"
    
#pragma config FNOSC = SPLL 
#pragma config FSOSCEN = OFF    
#pragma config POSCMOD = OFF    
#pragma config OSCIOFNC = ON        
#pragma config FPLLICLK = PLL_FRC   
#pragma config FPLLIDIV = DIV_1 
#pragma config FPLLMULT = MUL_50    
#pragma config FPLLODIV = DIV_2 
#pragma config FPLLRNG = RANGE_5_10_MHZ 
#pragma config FWDTEN = OFF           
#pragma config FDMTEN = OFF  
#pragma config DEBUG = OFF           
#pragma config JTAGEN = OFF         
#pragma config ICESEL = ICS_PGx1        
#pragma config TRCEN = ON        
#pragma config BOOTISA = MIPS32        
#pragma config FECCCON = OFF_UNLOCKED  
#pragma config FSLEEP = OFF            
#pragma config DBGPER = ALLOW_PG2      
#pragma config EJTAGBEN = NORMAL  
#pragma config PGL1WAY = OFF
#pragma config PMDL1WAY = OFF
#pragma config IOL1WAY = OFF

#define RemoteMax 15.0f

#ifdef micro
#define RGBled_red_pin 0
#define RGBled_green_pin 1
#define RGBled_blue_pin 2
#define motor_up_pin 6
#define motor_down_pin 4
#define motor_left_pin 3 
#define motor_right_pin 5
#endif
#ifdef mini
#define RGBled_red_pin 1
#define RGBled_green_pin 0
#define RGBled_blue_pin 2
#define motor_up_pin 6
#define motor_down_pin 4
#define motor_left_pin 3 
#define motor_right_pin 5
#endif
#ifdef big
#define RGBled_red_pin 9
#define RGBled_green_pin 10
#define RGBled_blue_pin 8
#define motor_up_pin 3
#define motor_down_pin 1
#define motor_left_pin 0 
#define motor_right_pin 2
#endif

#define PI 3.14159265
#define RadToDegrees 57.29577951

#define beta 1.0

#define MaxAltitudeBufferSize 3

#define MaxPitchRollTilt 20.0f // degrees
#define MaxYawRate 180.0f// degrees/sec
#define MaxAltitudeRate 1.0f// meters/sec

#define roll_offset 0 //degrees
#define pitch_offset 0 //degrees
#define heading_offset -87 //degrees

typedef struct{
    int up, down, left, right;
} Motors;  

typedef struct{
    float p, i, d;
    float error, sum, output;
} PID;

double difference_lat_lon(double, double, double, double);
float difference_bearing(double, double, double, double);
void main();
void menu(PID*, PID*, PID*, PID*, PID*, PID*);
void init();
void get_location();
void MadgwickQuaternionUpdate(float*, XYZ, XYZ, XYZ, float);
void str_write_int(int, unsigned char, char[], unsigned char);
void str_write_float(double, unsigned char, unsigned char, char[], unsigned char);
void delay_ms(unsigned int);
void timer2_init(float);
void timer3_init(float);
void timer4_init(float);
void timer5_init(float);
void timer6_init(float);
void limit_speed(Motors*);
void limit_angle(float*);
void write_RGB_led(unsigned int, unsigned int, unsigned int);
void PIDreset(PID*, float, float, float);
void Motorsreset(Motors*);

//------Timer counters------------
int loop_counter = 0;
unsigned long int delay_counter = 0;
//--------------------------------------Xbee----------------------------------------------
int remote_x1 = 0, remote_y1 = 0, remote_x2 = 0, remote_y2 = 0, safety_counter = 0;
unsigned char receive1, receive1_flag = 0, receive5, dial1, dial2, tx_buffer_counter = 0;
bool left_switch = 0, right_switch = 0, Xbee_signal = 0;
char tx_buffer[100];
//-----------------------------------------GPS--------------------------------------------
double latitude = 0.0, longitude = 0.0;
float GPS_altitude = 0.0;
unsigned int GPS_counter = 0;
char lat_str[16] = {'V', '\0'}, lon_str[16] = {'V', '\0'}, alt_str[6] = {'V', '\0'};
unsigned char GPS_stage = 0;
bool GPS_signal = 0;
//--------------------------------------------------------------------------------------------------------------------------------------
double difference_lat_lon(double lat1, double lon1, double lat2, double lon2){
    double a;
    lat1 /= RadToDegrees; lat2 /= RadToDegrees; lon1 /= RadToDegrees; lon2 /= RadToDegrees;
    a = pow(sin((lat2 - lat1) / 2), 2) + cos(lat1) * cos(lat2) * pow(sin((lon2 - lon1) / 2), 2);
    a = 2 * atan2(sqrt(a), sqrt(1 - a));
    return (6371000.0 * a);
}
float difference_bearing(double lat1, double lon1, double lat2, double lon2){
    double r;
    lat1 /= RadToDegrees; lat2 /= RadToDegrees; lon1 /= RadToDegrees; lon2 /= RadToDegrees;
    r = (atan2(-(sin(lon2 - lon1) * cos(lat2)), (cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1)))) * RadToDegrees;
    return r;
}
void multiply_vector_quarternion(float q[4], XYZ r, XYZ *v){
    v->x = r.x * (1-2*q[2]*q[2]-2*q[3]*q[3]) + r.y * (2*q[1]*q[2]-2*q[3]*q[0]) + r.z * (2*q[1]*q[3]+2*q[2]*q[0]);
    v->y = r.x * (2*q[1]*q[2]+2*q[3]*q[0]) + r.y * (1-2*q[1]*q[1]-2*q[3]*q[3]) + r.z * (2*q[2]*q[3]-2*q[1]*q[0]);
    v->z = r.x * (2*q[1]*q[3]-2*q[2]*q[0]) + r.y * (2*q[2]*q[3]+2*q[1]*q[0]) + r.z * (1-2*q[1]*q[1]-2*q[2]*q[2]);
}
//----------------------------------------------------------------------------------------------------------------------------------------
void main(){
    Motors speed;
    PID Pitch, Roll, Yaw, Altitude, GPS;
    PID Pitch_rate, Roll_rate, Yaw_rate, Altitude_rate;
    XYZ acc_comp; //Accleration values compensated for tilt, ie rotated from body frame to global frame
    int i;//General purpose loop counter
    unsigned char BMP_loop_timer;//Counter to for the BMP delays
    float q[4];//Quaternion
    float x_offset, y_offset, altitude_offset, take_off_altitude; //Offsets
    float heading, take_off_heading, yaw_offset, yaw_difference;//Yaw
    float remote_magnitude, remote_angle, remote_angle_difference; //RC
    float altitude_setpoint, acc_avg, altitude_buffer[MaxAltitudeBufferSize]; //Altitude
    double latitude_offset, longitude_offset, take_off_latitude, take_off_longitude; //GPS
    float GPS_bearing_difference; //GPS bearing relative to yaw
    double loop_time;
    char loop_mode, p_loop_mode;
    bool kill, p_kill;

    init();
    timer2_init(1000);// Delay timer - 1khz
    timer3_init(1000);// Safety timer for Xbee - 1khz
    timer4_init(1000000);// Loop timer - 1mhz
    timer5_init(6);// GPS timer - 6hz
    timer6_init(312500);// Xbee tx timer - 312.5khz
    USART1_init(111111);// Xbee
    USART5_init(9600);// GPS

    //Initializing all devices: MPU6050, HMC5883, PWM driver and the four ESCs
    delay_ms(100);
    MPU6050_init();
    HMC5883_init();
    BMP180_init();
    buffer_size = 0;
    
    pwm_driver_init(500);
    write_RGB_led(4095, 0, 0);
    if(remote_y2 > 29 && remote_x2 > 13){ //Calibrate ESCs
        write_pwm(motor_up_pin, 2006);
        write_pwm(motor_down_pin, 2006);
        write_pwm(motor_left_pin, 2006);
        write_pwm(motor_right_pin, 2006);
        delay_ms(500);
        write_pwm(motor_up_pin, 4013);
        write_pwm(motor_down_pin, 4013);
        write_pwm(motor_left_pin, 4013);
        write_pwm(motor_right_pin, 4013);
        delay_ms(4500);
    }
    write_pwm(motor_up_pin, 2006);
    write_pwm(motor_down_pin, 2006);
    write_pwm(motor_left_pin, 2006);
    write_pwm(motor_right_pin, 2006); 
    
    if(remote_y1 > 13 && remote_x1 > 13){ //display sensor readings
        T6CONbits.ON = 1;
        write_RGB_led(4095, 0, 3800);
        while(1){
            get_raw();
            if(tx_buffer_counter == 0){     
                tx_buffer[0] = 'D';
                str_write_int(acc.x, 6, tx_buffer, 1);
                str_write_int(acc.y, 6, tx_buffer, 8);
                str_write_int(acc.z, 6, tx_buffer, 15);
                str_write_int(gyro.x, 6, tx_buffer, 22);
                str_write_int(gyro.y, 6, tx_buffer, 29);
                str_write_int(gyro.z, 6, tx_buffer, 36);
                str_write_int(compass.x, 6, tx_buffer, 43);
                str_write_int(compass.y, 6, tx_buffer, 50);
                str_write_int(compass.z, 6, tx_buffer, 57);
                tx_buffer[64] = '\r';
                tx_buffer[65] = '\0';
                tx_buffer_counter++;
            }
        }
    }
    
    #ifdef micro
    PIDreset(&Roll, 1.5, 1.2, 0.0);
    PIDreset(&Pitch, 1.5, 1.2, 0.0);
    PIDreset(&Yaw, 1.5, 1.2, 0.0);
    PIDreset(&Roll_rate, 2.2, 0.0, 0.0);
    PIDreset(&Pitch_rate, 2.2, 0.0, 0.0);
    PIDreset(&Yaw_rate, 2.2, 0.0, 0.0);
    PIDreset(&Altitude, 1.8, 0.4, 0.0);
    PIDreset(&Altitude_rate, 40.0, 0.0, 0.0);
    PIDreset(&GPS, 1.5, 0.05, 0.0);
    #endif
    #ifdef mini
    PIDreset(&Roll, 1.5, 1.2, 0.0);
    PIDreset(&Pitch, 1.5, 1.2, 0.0);
    PIDreset(&Yaw, 2.0, 1.6, 0.0);
    PIDreset(&Roll_rate, 2.5, 0.0, 0.0);
    PIDreset(&Pitch_rate, 2.5, 0.0, 0.0);
    PIDreset(&Yaw_rate, 3.0, 0.0, 0.0);
    PIDreset(&Altitude, 1.8, 0.4, 0.0);
    PIDreset(&Altitude_rate, 40.0, 0.0, 0.0);
    PIDreset(&GPS, 1.5, 0.05, 0.0);
    #endif
    #ifdef big // 1.5, 1.2, 2.7
    PIDreset(&Roll, 1.2, 1.0, 0.0);
    PIDreset(&Pitch, 1.2, 1.0, 0.0);
    PIDreset(&Yaw, 1.2, 1.0, 0.0);
    PIDreset(&Roll_rate, 0.8, 0.0, 0.0);
    PIDreset(&Pitch_rate, 0.8, 0.0, 0.0);
    PIDreset(&Yaw_rate, 1.2, 0.0, 0.0);
    PIDreset(&Altitude, 1.8, 0.4, 0.0);
    PIDreset(&Altitude_rate, 40.0, 0.0, 0.0);
    PIDreset(&GPS, 1.5, 0.05, 0.0);
    #endif
    
    delay_ms(1500);
    
    while(1){
        //Clear PID variables
        PIDreset(&Roll, Roll.p, Roll.i, Roll.d);
        PIDreset(&Pitch, Pitch.p, Pitch.i, Pitch.d);
        PIDreset(&Yaw, Yaw.p, Yaw.i, Yaw.d);
        
        PIDreset(&Roll_rate, Roll_rate.p, Roll_rate.i, Roll_rate.d);
        PIDreset(&Pitch_rate, Pitch_rate.p, Pitch_rate.i, Pitch_rate.d);
        PIDreset(&Yaw_rate, Yaw_rate.p, Yaw_rate.i, Yaw_rate.d);
        
        PIDreset(&Altitude, Altitude.p, Altitude.i, Altitude.d);
        
        PIDreset(&GPS, GPS.p, GPS.i, GPS.d);

        //Reset quarternion
        q[0] = 1;
        q[1] = 0;
        q[2] = 0;
        q[3] = 0;
        
        //Clear motor speeds
        Motorsreset(&speed);
        
        yaw_offset = 0;
        
        BMP_loop_timer = 0;
        altitude_offset = 0;
        
        //Menu
        menu(&Roll, &Pitch, &Roll_rate, &Pitch_rate, &Altitude, &Altitude_rate);
        
        write_RGB_led(4095, 2500, 0);   
        
        T2CONbits.ON = 1;
        for(i = 0, acc_avg = 0, take_off_heading = 0; i < 1000; i++){
            delay_counter = 0;
            get_acc();
            get_gyro();
            get_compass();
            acc_avg += sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z) / 1000;
            MadgwickQuaternionUpdate(q, acc, gyro, compass, 0.005);
            USART1_send('B');
            USART1_send((i / 100 % 10) + 48);
            USART1_send((i / 10 % 10) + 48);
            USART1_send((i % 10) + 48);
            USART1_send('\r');
            while(delay_counter < 5);
        }
        T2CONbits.ON = 0;
        
        //Read take-off altitude
        for(i = 0, take_off_altitude = 0.0; i < MaxAltitudeBufferSize; i++){
            i2c5_write_registers(0xEE, (unsigned char[2]){0xF4, 0x2E}, 2);
            delay_ms(6);
            get_raw_temperature();//After 5ms read temperature
            i2c5_write_registers(0xEE, (unsigned char[2]){0xF4, 0x34 + oversampling * 64}, 2);//Initiate pressure read
            #if oversampling == 0 
            delay_ms(6);
            #elif oversampling == 1 
            delay_ms(9);
            #elif oversampling == 2   
            delay_ms(15);
            #elif oversampling == 3   
            delay_ms(27);
            #endif
            altitude_buffer[i] = get_altitude();
            take_off_altitude += altitude_buffer[i];
        }
        take_off_altitude /= MaxAltitudeBufferSize;
        
        //Read initial heading
        take_off_heading = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * RadToDegrees - heading_offset;
        
        if(GPS_signal){ take_off_latitude = latitude; take_off_longitude = longitude; }
        else{ take_off_latitude = 0; take_off_longitude = 0; }
        
        p_loop_mode = 0;
        p_kill = 0;
        loop_time = 0.00204;
        
        T4CONbits.ON = 1;//Start the loop timer
        T6CONbits.ON = 1;//Start the tx timer
        
        //Main Loop
        while(right_switch == 0){
            loop_counter = 0;
            
            //---------------------------------------------------------------Set Mode------------------------------------------------------------------------------
            
            if(left_switch) kill = 1; else kill = 0;
            if(dial1 == 0 || (dial1 == 2 && !GPS_signal)) loop_mode = 'S';
            else if(dial1 == 1) loop_mode = 'A';
            else if(dial1 == 2 && GPS_signal) loop_mode = 'P';
            if(p_loop_mode != loop_mode || p_kill != kill){
                if(kill) write_RGB_led(4095, 4095, 4095);
                else if(loop_mode == 'S') write_RGB_led(0, 4095, 0);
                else if(loop_mode == 'A'){
                    write_RGB_led(0, 4095, 4095);
                    altitude_setpoint = (float)remote_y2 * 2007.0f / 31.0f;
                    Altitude.sum = 0;
                    Altitude_rate.error = 0;
                    altitude_offset = Altitude.error;
                }
                else if(loop_mode == 'P'){
                    write_RGB_led(4095, 0, 4095);
                    latitude_offset = latitude;
                    longitude_offset = longitude;
                }
                yaw_offset = Yaw.error;
            }
            p_kill = kill;
            
            //--------------------------------------------------------IMU data acquisition---------------------------------------------------------------------------
            
            get_acc();
            get_gyro();
            get_compass(); 
            if(BMP_loop_timer == 0){
                i2c5_write_registers(0xEE, (unsigned char[2]){0xF4, 0x2E}, 2);//Initiate temperature read
            }
            else if(BMP_loop_timer == 3){
                get_raw_temperature();//After 5ms read temperature
                i2c5_write_registers(0xEE, (unsigned char[2]){0xF4, 0x34 + oversampling * 64}, 2);//Initiate pressure read
            }
            #if oversampling == 0 
            else if(BMP_loop_timer == 6){
            #elif oversampling == 1 
            else if(BMP_loop_timer == 7){
            #elif oversampling == 2   
            else if(BMP_loop_timer == 10){
            #elif oversampling == 3   
            else if(BMP_loop_timer == 16){
            #endif
                for(i = (MaxAltitudeBufferSize - 1); i >= 1; i--) altitude_buffer[i] = altitude_buffer[i - 1];
                altitude_buffer[0] = get_altitude();
                for(i = 0, Altitude.error = 0.0; i < MaxAltitudeBufferSize; i++) Altitude.error += altitude_buffer[i];
                Altitude.error = (Altitude.error / MaxAltitudeBufferSize) - take_off_altitude; 
                BMP_loop_timer = -1;
            }
            
            //----------------------------------------------------------------Filters----------------------------------------------------------------------------------
            
            MadgwickQuaternionUpdate(q, acc, gyro, compass, loop_time);//Update the quaternion with the current accelerometer, gyroscope and magnetometer vectors
            multiply_vector_quarternion(q, acc, &acc_comp);
            
            //Converting quaternion to Euler angles
            Roll.error = (atan2(2.0f * (q[0] * q[2] - q[3] * q[1]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) + PI) * RadToDegrees - roll_offset;
            Pitch.error = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) + PI) * RadToDegrees - pitch_offset;
            heading = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * RadToDegrees - heading_offset;
            limit_angle(&heading);
            Yaw.error = heading - take_off_heading;  
            limit_angle(&Roll.error);//Limit angles within -180 and +180 degrees
            limit_angle(&Pitch.error);
            limit_angle(&Yaw.error);
            yaw_difference = Yaw.error - yaw_offset;
            limit_angle(&yaw_difference);
            
             //------------------------------------------------Converting Remote data to a 2-D vector------------------------------------------------------------------
            
            if(loop_mode != 'P'){// If not in GPS mode
                remote_magnitude = sqrt((float)remote_x1 * (float)remote_x1 + (float)remote_y1 * (float)remote_y1); //Magnitude of remote's roll and pitch
                if(remote_x1 == 0 && remote_y1 == 0) remote_angle = 0;
                else remote_angle = -atan2((float)remote_x1, (float)remote_y1) * RadToDegrees;    //Angle with respect to pilot/starting position
                remote_angle_difference = Yaw.error - remote_angle;//Remote's angle with respect to quad's current direction
                limit_angle(&remote_angle_difference);
                y_offset = -MaxPitchRollTilt * remote_magnitude / RemoteMax * cos(remote_angle_difference / RadToDegrees);
                x_offset = MaxPitchRollTilt * remote_magnitude / RemoteMax * sin(remote_angle_difference / RadToDegrees);
            }
            
            //---------------------------------------------------------------3 modes------------------------------------------------------------------------------------
            
            if(loop_mode == 'S'){
                Altitude_rate.output = (float)remote_y2 * 2007.0f / 31.0f;
            }
            else if(loop_mode == 'A'){
                acc_comp.z = ((acc.z / (cos(Pitch.error / RadToDegrees) * cos(Roll.error / RadToDegrees)) + acc_avg) / 32768.0 * 2.0);
                if(acc_comp.z > 0.007 || acc_comp.z < (-0.007)) Altitude_rate.error += 10.0 * acc_comp.z * loop_time;                
                
                Altitude.output = (Altitude.p * (altitude_offset - Altitude.error) + Altitude.i * Altitude.sum);
                
                if(remote_y2 > 10 && remote_y2 < 20){
                    Altitude.sum += (altitude_offset - Altitude.error) * loop_time / 10;
                    Altitude_rate.output = (Altitude_rate.p * (Altitude_rate.error + Altitude.output));
                }
                else{
                    if(remote_y2 <= 10) Altitude_rate.output = (Altitude_rate.p * (Altitude_rate.error - 3.0 + Altitude.i * Altitude.sum));
                    else if(remote_y2 >= 20) Altitude_rate.output = (Altitude_rate.p * (Altitude_rate.error + 2.0 + Altitude.i * Altitude.sum));
                    altitude_offset = Altitude.error;                
                }
                //if(Altitude_rate.output < 0.0) Altitude_rate.output *= 1.2;
                Altitude_rate.output += altitude_setpoint;
            }
            
            else if(loop_mode == 'P'){
                Altitude_rate.output = (float)remote_y2 * 2007.0f / 31.0f;
                GPS.error = difference_lat_lon(take_off_latitude, take_off_longitude, latitude, longitude);
                GPS_bearing_difference = heading - difference_bearing(take_off_latitude, take_off_longitude, latitude, longitude);
                limit_angle(&GPS_bearing_difference);
                GPS.output = (GPS.p * GPS.error);
                y_offset = (-1) * GPS.output * cos((GPS_bearing_difference / RadToDegrees) + PI);
                x_offset = GPS.output * sin((GPS_bearing_difference / RadToDegrees) + PI);
                if(x_offset > 18) x_offset = 18; else if(x_offset < (-18)) x_offset = -18;
                if(y_offset > 18) y_offset = 18; else if(y_offset < (-18)) y_offset = -18;
            }
            p_loop_mode = loop_mode;
            
            //-----------------------------------------------------------------PID-------------------------------------------------------------------------------------
            
            /*------------
              Stab - PID
            ------------*/
            
            if(remote_y2 > 1 && !kill){
                Roll.sum += (Roll.error - x_offset) * loop_time * 3;
                Pitch.sum += (Pitch.error - y_offset) * loop_time * 3;
                Yaw.sum += (yaw_difference) * loop_time * 3;
            }
            
            Roll.output = (Roll.p * (Roll.error - x_offset) + Roll.i * Roll.sum);
            Pitch.output = (Pitch.p * (Pitch.error - y_offset) + Pitch.i * Pitch.sum);
            Yaw.output = (Yaw.p * (yaw_difference) + Yaw.i * Yaw.sum);
            
            /*------------
              Rate - PID
            ------------*/
            
            Roll_rate.error = gyro.y;//Roll.derivative;
            Pitch_rate.error = gyro.x;//Pitch.derivative;
            Yaw_rate.error = gyro.z;//Yaw.derivative;
            
            if(remote_y2 > 2 && !kill){
                Roll_rate.sum += (Roll_rate.error + Roll.output) * loop_time * 3;
                Pitch_rate.sum += (Pitch_rate.error + Pitch.output) * loop_time * 3;
                Yaw_rate.sum += (Yaw_rate.error + Yaw.output) * loop_time * 3;
            }
            
            Roll_rate.output = (Roll_rate.p * (Roll_rate.error + Roll.output)); //Roll PID
            Pitch_rate.output = (Pitch_rate.p * (Pitch_rate.error + Pitch.output)); //Pitch PID
            if(remote_x2 < 3 && remote_x2 > (-3)) Yaw_rate.output = (Yaw_rate.p * (Yaw_rate.error + Yaw.output)); //Yaw PID
            else{
                Yaw_rate.output = (Yaw_rate.p * (Yaw_rate.error + (float)remote_x2 / RemoteMax * MaxYawRate)); //Yaw PID
                Yaw.sum = 0;
                yaw_offset = Yaw.error;                
            }
            
            //-------------------------------------------------------------Motor Output--------------------------------------------------------------------------------
            
            speed.up = Altitude_rate.output - Pitch_rate.output + Roll_rate.output + Yaw_rate.output;
            speed.down = Altitude_rate.output + Pitch_rate.output - Roll_rate.output + Yaw_rate.output;
            speed.left = Altitude_rate.output - Pitch_rate.output - Roll_rate.output - Yaw_rate.output; 
            speed.right = Altitude_rate.output + Pitch_rate.output + Roll_rate.output - Yaw_rate.output;
            
            limit_speed(&speed);//Limit Motor output between 0 and 2054 and handle overflow
            
            //-----------------------------------------------------------Output to ESC's-------------------------------------------------------------------------------
            
            if(Xbee_signal && !kill){
                write_pwm(motor_up_pin, speed.up + 2006);
                write_pwm(motor_down_pin, speed.down + 2006);
                write_pwm(motor_left_pin, speed.left + 2006);
                write_pwm(motor_right_pin, speed.right + 2006);
            }
            else{
                write_pwm(motor_up_pin, 2006);
                write_pwm(motor_down_pin, 2006);
                write_pwm(motor_left_pin, 2006);
                write_pwm(motor_right_pin, 2006);
            }
            
            //--------------------------------------------------------Send Data to remote-----------------------------------------------------------------------------
            
            if(tx_buffer_counter == 0){     
                tx_buffer[0] = 'C';
                str_write_float(Roll.error, 3, 2, tx_buffer, 1);
                str_write_float(Pitch.error, 3, 2, tx_buffer, 8);
                str_write_float(Yaw.error, 3, 2, tx_buffer, 15);
                str_write_float(Altitude.error, 3, 2, tx_buffer, 22);
                str_write_float(latitude, 3, 8, tx_buffer, 29);
                str_write_float(longitude, 3, 8, tx_buffer, 42);
                tx_buffer[55] = loop_mode;
                tx_buffer[56] = '\r';
                tx_buffer[57] = '\0';
                tx_buffer_counter++;
            }
            
            //---------------------------------------------------------------------------------------------------------------------------------------------------------
            
            BMP_loop_timer++;
            
            while(loop_counter < 2040); //Loop should last at least 2.04 milliseconds
            loop_time = (double)loop_counter / 1000000; // Loop time in seconds:
        }
        //After killing and breaking from the main loop, stop all motors
        T4CONbits.ON = 0;//Stop the loop timer
        T6CONbits.ON = 0;
        write_pwm(motor_up_pin, 2006);
        write_pwm(motor_down_pin, 2006);
        write_pwm(motor_left_pin, 2006);
        write_pwm(motor_right_pin, 2006);
    }
}

void menu(PID *x, PID *y, PID *x_rate, PID *y_rate, PID *a, PID *a_rate){
    bool flag_menu = 1; signed char cursor = 0; unsigned int r, g, b, led_counter = 0, arming_counter = 0;
    while(arming_counter < 20){
        //-------------------------------------------------------------------LED stuff------------------------------------------------------------------------------
        if(led_counter < 4096){  r = 4095; g = led_counter; b = 0; }
        else if(led_counter < 8192){ r = 4095 - (led_counter - 4096); g = 4095; b = 0; }
        else if(led_counter < 12288){ r = 0; g = 4095; b = led_counter - 8192; }
        else if(led_counter < 16384){ r = 0; g = 4095 - (led_counter - 12288); b = 4095; }
        else if(led_counter < 20480){ r = led_counter - 16384; g = 0; b = 4095; }
        else{ r = 4095; g = 0; b = 4095 - (led_counter - 20480); }
        led_counter += 150;
        if(led_counter >= 24576) led_counter = 0;
        write_RGB_led(r, g, b);
        //----------------------------------------------------------------------------------------------------------------------------------------------------------
        if(remote_y1 > 12 && flag_menu == 0){ cursor--; flag_menu = 1; }
        else if(remote_y1 < (-12) && flag_menu == 0){ cursor++; flag_menu = 1; }
        else if(remote_y1 > (-12) && remote_y1 < 12) flag_menu = 0;
        if(cursor > 5) cursor = 5;
        else if(cursor < 0) cursor = 0;

        if(cursor == 0){ if(remote_x1 > 12){ x->p += 0.01; y->p += 0.01; }else if(remote_x1 < (-12)){ x->p -= 0.01; y->p -= 0.01; }}
        if(cursor == 1){ if(remote_x1 > 12){ x->i += 0.01; y->i += 0.01; }else if(remote_x1 < (-12)){ x->i -= 0.01; y->i -= 0.01; }}
        if(cursor == 2){ if(remote_x1 > 12){ x_rate->p += 0.01; y_rate->p += 0.01; }else if(remote_x1 < (-12)){ x_rate->p -= 0.01; y_rate->p -= 0.01; }}
        if(cursor == 3){ if(remote_x1 > 12) a->p += 0.2; else if(remote_x1 < (-12)) a->p -= 0.2; }
        if(cursor == 4){ if(remote_x1 > 12) a->i += 0.01; else if(remote_x1 < (-12)) a->i -= 0.01; }
        if(cursor == 5){ if(remote_x1 > 12) a_rate->p += 1; else if(remote_x1 < (-12)) a_rate->p -= 1; }
        
        if(left_switch && !right_switch && remote_y2 < 2 && remote_x2 > 13) arming_counter++;
        else arming_counter = 0;
        
        delay_counter = 0; T2CONbits.TON = 1;
        while(delay_counter < 25){
            if(receive1_flag){
                get_gyro();
                USART1_send('A');
                USART1_send((cursor % 10) + 48);
                USART1_write_float(x->p, 2, 2);
                USART1_write_float(x->i, 2, 2);
                USART1_write_float(x_rate->p, 2, 2);
                USART1_write_float(a->p, 3, 1);
                USART1_write_float(a->i, 2, 2);
                USART1_write_float(a_rate->p, 3, 1);
                USART1_send((GPS_signal % 10) + 48);
                USART1_send(((arming_counter / 10) % 10) + 48);
                USART1_send((arming_counter % 10) + 48);
                USART1_send('\r');
                break;
            }
        }
        while(delay_counter < 25); T2CONbits.TON = 0;
    }
}

void init(){
    TRISB = 0; TRISC = 0; TRISD = 0; TRISE = 0; TRISF = 0;
    ANSELB = 0;
    LATBbits.LATB15 = 0;
    TRISBbits.TRISB15 = 1;
    
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
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
void __ISR_AT_VECTOR(_UART1_RX_VECTOR, IPL6SRS) Xbee_rx(void){
    IFS3bits.U1RXIF = 0; 
    do{
        receive1 = U1RXREG & 0xFF;
        if(receive1 >> 5 == 0) remote_x1 = (receive1 & 0x1F) - 15;
        else if(receive1 >> 5 == 1) remote_y1 = (receive1 & 0x1F) - 15;
        else if(receive1 >> 5 == 2) remote_x2 = (receive1 & 0x1F) - 15;
        else if(receive1 >> 5 == 3) remote_y2 = (receive1 & 0x1F);
        else if(receive1 >> 5 == 4){ left_switch = (receive1 >> 1) & 1; right_switch = receive1 & 1; }
        else if(receive1 >> 5 == 5){
            dial2 = (receive1 & 0b00001100) << 2;
            dial1 = (receive1 & 0b00000011);
            safety_counter = 0;
            Xbee_signal = 1;
            receive1_flag = 1;
        }
    }while(U1STAbits.URXDA);
    IFS3bits.U1RXIF = 0; 
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
void __ISR_AT_VECTOR(_UART5_RX_VECTOR, IPL6SRS) GPS_(void){
    IFS5bits.U5RXIF = 0; TMR5 = 0;
    do{ receive5 = U5RXREG & 0xFF;
        if(GPS_stage == 0){ if(receive5 == '$') GPS_stage++;}
        else if(GPS_stage == 1){ if(receive5 == 'G') GPS_stage++; else GPS_stage = 0; }
        else if(GPS_stage == 2){ if(receive5 == 'P') GPS_stage++; else GPS_stage = 0; }
        else if(GPS_stage == 3){ if(receive5 == 'G') GPS_stage++; else GPS_stage = 0; }
        else if(GPS_stage == 4){ if(receive5 == 'G') GPS_stage++; else GPS_stage = 0; }
        else if(GPS_stage == 5){ if(receive5 == 'A') GPS_stage++; else GPS_stage = 0; }
        else if(GPS_stage == 6 && receive5 == ','){ GPS_stage++; }  
        else if(GPS_stage == 7 && receive5 == ','){ USART1_send(receive5);GPS_stage++; GPS_counter = 0; }
        else if(GPS_stage == 8){ if(receive5 == ','){ GPS_stage++; lat_str[GPS_counter] = '\0'; } else lat_str[GPS_counter++] = receive5; }
        else if(GPS_stage == 9 && receive5 == ','){ GPS_stage++; GPS_counter = 0; }    
        else if(GPS_stage == 10){ if(receive5 == ','){ GPS_stage++; lon_str[GPS_counter] = '\0'; } else lon_str[GPS_counter++] = receive5; }
        else if(GPS_stage == 11 && receive5 == ','){ GPS_stage++; }
        else if(GPS_stage == 12){ if(receive5 == '0'){ GPS_signal = 0; GPS_stage = 0; }else{ GPS_stage++; GPS_counter = 0; GPS_signal = 1; }}
        else if(GPS_stage == 13 && receive5 == ','){ GPS_stage++; }
        else if(GPS_stage == 14 && receive5 == ','){ GPS_stage++; }
        else if(GPS_stage == 15 && receive5 == ','){ GPS_stage++; GPS_counter = 0; }
        else if(GPS_stage == 16){ if(receive5 == ','){ GPS_stage = 0; alt_str[GPS_counter] = '\0'; } else alt_str[GPS_counter++] = receive5; }
    }while(U5STAbits.URXDA); IFS5bits.U5RXIF = 0; 
}
void get_location(){
    int left, i;
    double temp_left, temp_right, tens;
    if(!GPS_signal){ latitude = 0; longitude = 0; return; }
    for(i = 0; lat_str[i] != '.'; i++); left = i - 2;
    for(i = left - 1, tens = 1.0, temp_left = 0.0; i >= 0; i--, tens *= 10.0) temp_left += (double)(lat_str[i] - 48) * tens;
    for(i = left, tens = 10.0, temp_right = 0.0; lat_str[i] != '\0'; i++){ if(lat_str[i] != '.'){ temp_right += (double)(lat_str[i] - 48) * tens; tens /= 10; }}
    latitude = temp_left + (temp_right / 60.0);
    for(i = 0; lon_str[i] != '.'; i++); left = i - 2;
    for(i = left - 1, tens = 1.0, temp_left = 0.0; i >= 0; i--, tens *= 10.0) temp_left += (double)(lon_str[i] - 48) * tens;
    for(i = left, tens = 10.0, temp_right = 0.0; lon_str[i] != '\0'; i++){ if(lon_str[i] != '.'){ temp_right += (double)(lon_str[i] - 48) * tens; tens /= 10; }}
    longitude = temp_left + (temp_right / 60.0);
    for(i = 0; alt_str[i] != '.'; i++); left = i;
    for(i = left - 1, tens = 1.0, temp_left = 0.0; i >= 0; i--, tens *= 10.0) temp_left += (float)(alt_str[i] - 48) * tens;
    for(i = left + 1, tens = 10.0, temp_right = 0.0; alt_str[i] != '\0'; i++, tens *= 10.0) temp_right += (float)(alt_str[i] - 48) / tens;
    GPS_altitude = temp_left + temp_right;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
void MadgwickQuaternionUpdate(float q[], XYZ acc, XYZ gy, XYZ mag, float deltat){
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; XYZ a, g, m;
    float norm, hx, hy, _2bx, _2bz, s1, s2, s3, s4, qDot1, qDot2, qDot3, qDot4, _2q1mx, _2q1my, _2q1mz, _2q2mx, _4bx, _4bz;
    float _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3, _2q4 = 2.0f * q4, _2q1q3 = 2.0f * q1 * q3, _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3, q1q4 = q1 * q4, q2q2 = q2 * q2, q2q3 = q2 * q3, q2q4 = q2 * q4, q3q3 = q3 * q3, q3q4 = q3 * q4, q4q4 = q4 * q4;
    norm = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z); a.x = acc.x / norm; a.y = acc.y / norm; a.z = acc.z / norm;
    g.x = gy.x / RadToDegrees; g.y = gy.y / RadToDegrees; g.z = gy.z / RadToDegrees; 
    norm = sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);m.x = mag.x / norm; m.y = mag.y / norm; m.z = mag.z / norm;
    _2q1mx = 2.0f * q1 * m.x; _2q1my = 2.0f * q1 * m.y; _2q1mz = 2.0f * q1 * m.z; _2q2mx = 2.0f * q2 * m.x;
    hx = m.x * q1q1 - _2q1my * q4 + _2q1mz * q3 + m.x * q2q2 + _2q2 * m.y * q3 + _2q2 * m.z * q4 - m.x * q3q3 - m.x * q4q4;
    hy = _2q1mx * q4 + m.y * q1q1 - _2q1mz * q2 + _2q2mx * q3 - m.y * q2q2 + m.y * q3q3 + _2q3 * m.z * q4 - m.y * q4q4;
    _2bx = sqrt(hx * hx + hy * hy); _2bz = -_2q1mx * q3 + _2q1my * q2 + m.z * q1q1 + _2q2mx * q4 - m.z * q2q2 + _2q3 * m.y * q4 - m.z * q3q3 + m.z * q4q4; _4bx = 2.0f * _2bx; _4bz = 2.0f * _2bz;
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - a.x) + _2q2 * (2.0f * q1q2 + _2q3q4 - a.y) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m.x) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m.y) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m.z);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - a.x) + _2q1 * (2.0f * q1q2 + _2q3q4 - a.y) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - a.z) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m.x) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m.y) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m.z);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - a.x) + _2q4 * (2.0f * q1q2 + _2q3q4 - a.y) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - a.z) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m.x) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m.y) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m.z);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - a.x) + _2q3 * (2.0f * q1q2 + _2q3q4 - a.y) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m.x) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m.y) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m.z);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); s1 /= norm; s2 /= norm; s3 /= norm; s4 /= norm;
    qDot1 = 0.5f * (-q2 * g.x - q3 * g.y - q4 * g.z) - beta * s1; qDot2 = 0.5f * (q1 * g.x + q3 * g.z - q4 * g.y) - beta * s2; qDot3 = 0.5f * (q1 * g.y - q2 * g.z + q4 * g.x) - beta * s3; qDot4 = 0.5f * (q1 * g.z + q2 * g.y - q3 * g.x) - beta * s4;
    q1 += qDot1 * deltat; q2 += qDot2 * deltat; q3 += qDot3 * deltat; q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); 
    q[0] = q1 / norm; q[1] = q2 / norm; q[2] = q3 / norm; q[3] = q4 / norm;
}
void str_write_int(int a, unsigned char precision, char str[], unsigned char n){
    if(a < 0){ a *= (-1); str[n++] = '-'; }
    else str[n++] = '+';
    if(precision >= 6) str[n++] = ((a / 100000) % 10) + 48;
    if(precision >= 5) str[n++] = ((a / 10000) % 10) + 48;
    if(precision >= 4) str[n++] = ((a / 1000) % 10) + 48;
    if(precision >= 3) str[n++] = ((a / 100) % 10) + 48;
    if(precision >= 2) str[n++] = ((a / 10) % 10) + 48;
    if(precision >= 1) str[n++] = (a % 10) + 48;
}
void str_write_float(double a, unsigned char left, unsigned char right, char str[], unsigned char n){
    unsigned char i;
    long int tens = 10;
    if(a < 0){ a *= (-1); str[n++] = '-'; }
    else str[n++] = '+';
    if(left >= 7) str[n++] = ((int)(a / 1000000) % 10) + 48;
    if(left >= 6) str[n++] = ((int)(a / 100000) % 10) + 48;
    if(left >= 5) str[n++] = ((int)(a / 10000) % 10) + 48;
    if(left >= 4) str[n++] = ((int)(a / 1000) % 10) + 48;
    if(left >= 3) str[n++] = ((int)(a / 100) % 10) + 48;
    if(left >= 2) str[n++] = ((int)(a / 10) % 10) + 48;
    if(left >= 1) str[n++] = ((int)a % 10) + 48;
    str[n++] = '.';
    for(i = 0; i < right; i++, tens *= 10, n++) str[n] = ((long int)(a * tens) % 10) + 48;
}
//-------------------------------------------------------------------
void __ISR_AT_VECTOR(_TIMER_2_VECTOR, IPL4SRS) delay_timer(void){
    IFS0bits.T2IF = 0;
    delay_counter++;
}
void delay_ms(unsigned int x){
    delay_counter = 0;
    T2CONbits.ON = 1;
    while(delay_counter < x);
    T2CONbits.ON = 0;
}
void timer2_init(float frequency){
    float t = 100000000.0 / frequency; unsigned char pre = 0;
    while(t > 65535){ t /= 2.0; pre++; }
    t = (int)t;
    while((int)t % 2 == 0 && pre < 8){ t /= 2.0; pre++; }
    if(pre == 7){ t *= 2.0; pre--; }
    if(pre == 8) pre = 7;
    T2CONbits.ON = 0;
    T2CONbits.TCKPS = pre;
    PR2 = (int)t - 1;
    TMR2 = 0;
    IPC2bits.T2IP = 4;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
}
//--------------------------------------------------------------------
void __ISR_AT_VECTOR(_TIMER_3_VECTOR, IPL4SRS) safety_timer(void){
    IFS0bits.T3IF = 0;
    if(safety_counter == 10) receive1_flag = 0;
    if(safety_counter == 500){ Xbee_signal = 0; remote_x1 = 0; remote_y1 = 0; remote_x2 = 0; remote_y2 = 0; }
    else safety_counter++;
}
void timer3_init(float frequency){
    float t = 100000000.0 / frequency; unsigned char pre = 0;
    while(t > 65535){ t /= 2.0; pre++; }
    t = (int)t;
    while((int)t % 2 == 0 && pre < 8){ t /= 2.0; pre++; }
    if(pre == 7){ t *= 2.0; pre--; }
    if(pre == 8) pre = 7;
    T3CONbits.ON = 0;
    T3CONbits.TCKPS = pre;
    PR3 = (int)t - 1;
    TMR3 = 0;
    IPC3bits.T3IP = 4;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    T3CONbits.ON = 1;
}
//-------------------------------------------------------------------
void __ISR_AT_VECTOR(_TIMER_4_VECTOR, IPL7SRS) pid_loop_timer(void){
    IFS0bits.T4IF = 0;
    loop_counter++;
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
}
//-------------------------------------------------------------------
void __ISR_AT_VECTOR(_TIMER_5_VECTOR, IPL4SRS) GPS_timer(void){
    IFS0bits.T5IF = 0;
    get_location();
}
void timer5_init(float frequency){
    float t = 100000000.0 / frequency; unsigned char pre = 0;
    while(t > 65535){ t /= 2.0; pre++; }
    t = (int)t;
    while((int)t % 2 == 0 && pre < 8){ t /= 2.0; pre++; }
    if(pre == 7){ t *= 2.0; pre--; }
    if(pre == 8) pre = 7;
    T5CONbits.ON = 0;
    T5CONbits.TCKPS = pre;
    PR5 = (int)t - 1;
    TMR5 = 0;
    IPC6bits.T5IP = 4;
    IFS0bits.T5IF = 0;
    IEC0bits.T5IE = 1;
    T5CONbits.ON = 1;
}
//-------------------------------------------------------------------
void __ISR_AT_VECTOR(_TIMER_6_VECTOR, IPL4SRS) transmit_timer(void){
    IFS0bits.T6IF = 0;
    if(tx_buffer_counter > 0 && receive1_flag){
        while(!U1STAbits.UTXBF){
            if(tx_buffer[tx_buffer_counter - 1] == '\0'){
                tx_buffer_counter = 0;
                break;
            }
            U1TXREG = tx_buffer[(tx_buffer_counter++ - 1)];
        }
    }
}
void timer6_init(float frequency){
    float t = 100000000.0 / frequency; unsigned char pre = 0;
    while(t > 65535){ t /= 2.0; pre++; }
    t = (int)t;
    while((int)t % 2 == 0 && pre < 8){ t /= 2.0; pre++; }
    if(pre == 7){ t *= 2.0; pre--; }
    if(pre == 8) pre = 7;
    T6CONbits.ON = 0;
    T6CONbits.TCKPS = pre;
    PR6 = (int)t - 1;
    TMR6 = 0;
    IPC7bits.T6IP = 4;
    IFS0bits.T6IF = 0;
    IEC0bits.T6IE = 1;
}
//-------------------------------------------------------------------
void write_RGB_led(unsigned int r, unsigned int g, unsigned int b){
    write_pwm(RGBled_red_pin, r); write_pwm(RGBled_green_pin, g); write_pwm(RGBled_blue_pin, b);
}
void limit_angle(float *a){
    while(*a < (-180) || *a > 180){ if(*a < (-180)) *a += 360; else if(*a > 180) *a -= 360; }
}
void limit_speed(Motors *x){
    if(x->up < 0) x->up = 0;
    else if(x->up > 2006) x->up = 2006;
    if(x->down < 0) x->down = 0;
    else if(x->down > 2006) x->down = 2006;
    if(x->right < 0) x->right = 0;
    else if(x->right > 2006) x->right = 2006;
    if(x->left < 0) x->left = 0;
    else if(x->left > 2006) x->left = 2006;
}
void PIDreset(PID *x, float kp, float ki, float kd){
    x->p = kp; x->i = ki; x->d = kd; x->sum = 0.0f; x->output = 0.0f; x->error = 0.0f;
}
void Motorsreset(Motors *x){
    x->up = 0; x->down = 0; x->left = 0; x->right = 0;
}