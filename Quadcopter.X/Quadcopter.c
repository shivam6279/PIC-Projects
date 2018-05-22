#include "pragma.h"
#include <xc.h>
#include <math.h>
#include <stdbool.h>
#include <sys/attribs.h>
#include "settings.h"
#include "pic32.h"
#include "bitbang_i2c.h"
#include "PWMDriver.h"
#include "10DOF.h"
#include "USART.h"
#include "XBee.h"
#include "PID.h"
#include "motor.h"
#include "GPS.h"
#include "AHRS.h"
#include "init.h"
#include "menu.h"

#include "GPS_ISR.h"
#include "XBee_ISR.h"
#include "timer_ISR.h"

#ifdef micro
#warning "--------------------------------Building for Micro quad!--------------------------------"
#endif
#ifdef mini
#warning "--------------------------------Building for Mini quad!--------------------------------"
#endif
#ifdef big
#warning "--------------------------------Building for Big quad!--------------------------------"
#endif

#define REMOTE_MAX 15.0f
#define THROTTLE_MAX 31.0f

#define MAX_PITCH_ROLL_TILT 20.0f // degrees
#define MaxYawRate 180.0f// degrees/sec
#define MaxAltitudeRate 1.0f// meters/sec

#define ROLLOFFSET 0 //degrees
#define PITCHOFFSET 0 //degrees
#define HEADINGOFFSET -87 //degrees

#define ALTITUDE_BUFFER_SIZE 3

void main(){
    Motors speed;
    PID pitch, roll, yaw, altitude, GPS;
    PID pitch_rate, roll_rate, yaw_rate, altitude_rate;
    XYZ acc_comp;                                                                   //Til-compensated acceleration
    int i;                                                                          //General purpose loop counter
    unsigned char BMP_loop_timer;                                                   //Counter to for the BMP delays
    float q[4];                                                                     //Quaternion
    float take_off_altitude;                                                        //Offsets
    float heading, take_off_heading, yaw_difference;                                //yaw
    float remote_magnitude, remote_angle, remote_angle_difference;                  //RC
    float altitude_setpoint, acc_avg, altitude_buffer[ALTITUDE_BUFFER_SIZE];        //altitude
    double latitude_offset, longitude_offset, take_off_latitude, take_off_longitude;//GPS
    float GPS_bearing_difference;                                                   //GPS bearing relative to yaw
    double loop_time;                                                               //Main loop time
    char loop_mode, p_loop_mode;                                                    //Stabilize/alt-hold/pos-hold
    bool kill, p_kill;
    
    Init();
    
    WriteRGBLed(4095, 0, 0);    //Red
    if(remote_y2 > 29 && remote_x2 > 13) { //Calibrate ESCs
        CalibrateESC();
    }
    TurnMotorsOff();
    
    if(remote_y1 > 13 && remote_x1 > 13) { //display sensor readings
        T6CONbits.ON = 1;
        WriteRGBLed(4095, 0, 3800); //Purple
        while(1) {
            SendCalibrationData();
        }
    }
    
    #ifdef micro
    PIDSet(&roll, 1.5, 1.2, 0.0);
    PIDSet(&pitch, 1.5, 1.2, 0.0);
    PIDSet(&yaw, 1.5, 1.2, 0.0);
    PIDSet(&roll_rate, 2.2, 0.0, 0.0);
    PIDSet(&pitch_rate, 2.2, 0.0, 0.0);
    PIDSet(&yaw_rate, 2.2, 0.0, 0.0);
    PIDSet(&altitude, 1.8, 0.4, 0.0);
    PIDSet(&altitude_rate, 40.0, 0.0, 0.0);
    PIDSet(&GPS, 1.5, 0.05, 0.0);
    #endif
    #ifdef mini
    PIDSet(&roll, 1.5, 1.2, 0.0);
    PIDSet(&pitch, 1.5, 1.2, 0.0);
    PIDSet(&yaw, 2.0, 1.6, 0.0);
    PIDSet(&roll_rate, 2.5, 0.0, 0.0);
    PIDSet(&pitch_rate, 2.5, 0.0, 0.0);
    PIDSet(&yaw_rate, 3.0, 0.0, 0.0);
    PIDSet(&altitude, 1.8, 0.4, 0.0);
    PIDSet(&altitude_rate, 40.0, 0.0, 0.0);
    PIDSet(&GPS, 1.5, 0.05, 0.0);
    #endif
    #ifdef big // 1.5, 1.2, 2.7
    PIDSet(&roll, 1.2, 1.0, 0.0);
    PIDSet(&pitch, 1.2, 1.0, 0.0);
    PIDSet(&yaw, 1.2, 1.0, 0.0);
    PIDSet(&roll_rate, 0.8, 0.0, 0.0);
    PIDSet(&pitch_rate, 0.8, 0.0, 0.0);
    PIDSet(&yaw_rate, 1.2, 0.0, 0.0);
    PIDSet(&altitude, 1.8, 0.4, 0.0);
    PIDSet(&altitude_rate, 40.0, 0.0, 0.0);
    PIDSet(&GPS, 1.5, 0.05, 0.0);
    #endif
    
    delay_ms(1500);
    
    while(1) {
        ResetPID(&roll, &pitch, &yaw, &roll_rate, &pitch_rate, &yaw_rate, &altitude, &altitude_rate, &GPS);//Clear PID variables
        ResetQuaternion(q); //Reset quarternion
        MotorsReset(&speed);//Clear motor speeds
        altitude.offset = 0;
        
        //Menu
        menu(&roll, &pitch, &roll_rate, &pitch_rate, &altitude, &altitude_rate);
        
        WriteRGBLed(4095, 2500, 0); //Yellow
        
        T2CONbits.ON = 1;// Turn on the delay timer @ 1kHz
        for(i = 0, acc_avg = 0, take_off_heading = 0; i < 1000; i++) {
            delay_counter = 0;
            GetAcc();
            GetGyro();
            GetCompass();
            acc_avg += sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z) / 1000;
            MadgwickQuaternionUpdate(q, acc, gyro, compass, 0.005);
            USART1_send('B');
            USART1_send((i / 100 % 10) + 48);
            USART1_send((i / 10 % 10) + 48);
            USART1_send((i % 10) + 48);
            USART1_send('\r');
            while(delay_counter < 5);
        }
        T2CONbits.ON = 0;// Turn off the delay timer
        
        //Read take-off altitude
        for(i = 0, take_off_altitude = 0.0; i < ALTITUDE_BUFFER_SIZE; i++) {
            i2c5_write_registers(0xEE, (unsigned char[2]){0xF4, 0x2E}, 2);
            delay_ms(6);
            GetRawTemperature();//After 5ms read temperature
            i2c5_write_registers(0xEE, (unsigned char[2]){0xF4, 0x34 + OVERSAMPLING * 64}, 2);//Initiate pressure read
            #if OVERSAMPLING == 0 
            delay_ms(6);
            #elif OVERSAMPLING == 1 
            delay_ms(9);
            #elif OVERSAMPLING == 2   
            delay_ms(15);
            #elif OVERSAMPLING == 3   
            delay_ms(27);
            #endif
            altitude_buffer[i] = GetAltitude();
            take_off_altitude += altitude_buffer[i];
        }
        take_off_altitude /= ALTITUDE_BUFFER_SIZE;
        
        //Read initial heading
        take_off_heading = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * RAD_TO_DEGREES - HEADINGOFFSET;
        
        if(GPS_signal) { 
            take_off_latitude = latitude; 
            take_off_longitude = longitude; 
        } else { 
            take_off_latitude = 0.0; 
            take_off_longitude = 0.0; 
        }
        
        p_loop_mode = 0;
        p_kill = 0;
        loop_time = 0.00204;
        BMP_loop_timer = 0;
        
        T4CONbits.ON = 1;   //Start the loop timer
        T6CONbits.ON = 1;   //Start the tx timer
        
        //Main Loop
        while(right_switch == 0) {
            loop_counter = 0;
            
            //---------------------------------------------------------------Set Mode------------------------------------------------------------------------------
            
            if(left_switch) kill = 1; 
            else kill = 0;
            if(dial1 == 0 || (dial1 == 2 && !GPS_signal)) loop_mode = 'S';
            else if(dial1 == 1) loop_mode = 'A';
            else if(dial1 == 2 && GPS_signal) loop_mode = 'P';
            if(p_loop_mode != loop_mode || p_kill != kill) {
                if(kill) WriteRGBLed(4095, 4095, 4095);
                else if(loop_mode == 'S') WriteRGBLed(0, 4095, 0);
                else if(loop_mode == 'A') {
                    WriteRGBLed(0, 4095, 4095);
                    altitude_setpoint = (float)remote_y2 * (MOTOR_MAX - MOTOR_OFF) / THROTTLE_MAX;
                    altitude.sum = 0;
                    altitude_rate.error = 0;
                    altitude.offset = altitude.error;
                }
                else if(loop_mode == 'P'){
                    WriteRGBLed(4095, 0, 4095);
                    latitude_offset = latitude;
                    longitude_offset = longitude;
                }
                yaw.offset = yaw.error;
            }
            p_kill = kill;
            
            //--------------------------------------------------------IMU data acquisition---------------------------------------------------------------------------
            
            GetAcc();
            GetGyro();
            GetCompass(); 
            if(BMP_loop_timer == 0) {
                i2c5_write_registers(0xEE, (unsigned char[2]){0xF4, 0x2E}, 2);//Initiate temperature read
            }
            else if(BMP_loop_timer == 3) {
                GetRawTemperature();//After 5ms read temperature
                i2c5_write_registers(0xEE, (unsigned char[2]){0xF4, 0x34 + OVERSAMPLING * 64}, 2);//Initiate pressure read
            }
            #if OVERSAMPLING == 0 
            else if(BMP_loop_timer == 6) {
            #elif OVERSAMPLING == 1 
            else if(BMP_loop_timer == 7) {
            #elif OVERSAMPLING == 2   
            else if(BMP_loop_timer == 10) {
            #elif OVERSAMPLING == 3   
            else if(BMP_loop_timer == 16) {
            #endif
                for(i = (ALTITUDE_BUFFER_SIZE - 1); i >= 1; i--) altitude_buffer[i] = altitude_buffer[i - 1];
                altitude_buffer[0] = GetAltitude();
                for(i = 0, altitude.error = 0.0; i < ALTITUDE_BUFFER_SIZE; i++) altitude.error += altitude_buffer[i];
                altitude.error = (altitude.error / ALTITUDE_BUFFER_SIZE) - take_off_altitude; 
                BMP_loop_timer = -1;
            }
            
            //----------------------------------------------------------------Filters----------------------------------------------------------------------------------
            
            MadgwickQuaternionUpdate(q, acc, gyro, compass, loop_time); //Update the quaternion with the current accelerometer, gyroscope and magnetometer vectors
            MultiplyVectorQuarternion(q, acc, &acc_comp);
            
            //Converting quaternion to Euler angles
            roll.error = (atan2(2.0f * (q[0] * q[2] - q[3] * q[1]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) + PI) * RAD_TO_DEGREES - ROLLOFFSET;
            pitch.error = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) + PI) * RAD_TO_DEGREES - PITCHOFFSET;
            heading = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * RAD_TO_DEGREES - HEADINGOFFSET;
            LimitAngle(&heading);
            yaw.error = heading - take_off_heading;  
            LimitAngle(&roll.error);//Limit angles within -180 and +180 degrees
            LimitAngle(&pitch.error);
            LimitAngle(&yaw.error);
            yaw_difference = yaw.error - yaw.offset;
            LimitAngle(&yaw_difference);
            
            //------------------------------------------------Converting Remote data to a 2-D vector------------------------------------------------------------------
            
            if(loop_mode != 'P') {// If not in GPS mode
                remote_magnitude = sqrt((float)remote_x1 * (float)remote_x1 + (float)remote_y1 * (float)remote_y1); //Magnitude of remote's roll and pitch
                if(remote_x1 == 0 && remote_y1 == 0) remote_angle = 0;
                else remote_angle = -atan2((float)remote_x1, (float)remote_y1) * RAD_TO_DEGREES;  //Angle with respect to pilot/starting position
                remote_angle_difference = yaw.error - remote_angle; //Remote's angle with respect to quad's current direction
                LimitAngle(&remote_angle_difference);
                pitch.offset = -MAX_PITCH_ROLL_TILT * remote_magnitude / REMOTE_MAX * cos(remote_angle_difference / RAD_TO_DEGREES);
                roll.offset = MAX_PITCH_ROLL_TILT * remote_magnitude / REMOTE_MAX * sin(remote_angle_difference / RAD_TO_DEGREES);
            }
            
            //---------------------------------------------------------------3 modes------------------------------------------------------------------------------------
            
            if(loop_mode == 'S') {
                altitude_rate.output = (float)remote_y2 * (MOTOR_MAX - MOTOR_OFF) / THROTTLE_MAX;
            }
            else if(loop_mode == 'A') {
                acc_comp.z = ((acc.z / (cos(pitch.error / RAD_TO_DEGREES) * cos(roll.error / RAD_TO_DEGREES)) + acc_avg) / 32768.0 * 2.0);
                //if(acc_comp.z > 0.007 || acc_comp.z < (-0.007)) altitude_rate.error += 10.0 * acc_comp.z * loop_time;                
                
                if(remote_y2 > 10 && remote_y2 < 20) {  //Throttle stick in the mid position
                    altitude.sum += (altitude.offset - altitude.error) * loop_time / 10;
                    altitude_rate.output = (altitude_rate.p * (altitude_rate.error + altitude.output));
                } else {                                //Throttle stick either up or down
                    if(remote_y2 <= 10) altitude_rate.output = (altitude_rate.p * (altitude_rate.error - 3.0));
                    else if(remote_y2 >= 20) altitude_rate.output = (altitude_rate.p * (altitude_rate.error + 3.0));
                    altitude.offset = altitude.error;                
                }
                altitude.output = (altitude.p * (altitude.offset - altitude.error) + altitude.i * altitude.sum);
                //if(altitude_rate.output < 0.0) altitude_rate.output *= 1.2;
                altitude_rate.output += altitude_setpoint;
            }
            
            else if(loop_mode == 'P') {
                altitude_rate.output = (float)remote_y2 * (MOTOR_MAX - MOTOR_OFF) / THROTTLE_MAX;
                GPS.error = DifferenceLatLon(take_off_latitude, take_off_longitude, latitude, longitude);
                GPS_bearing_difference = heading - DifferenceBearing(take_off_latitude, take_off_longitude, latitude, longitude);
                LimitAngle(&GPS_bearing_difference);
                GPS.output = (GPS.p * GPS.error);
                pitch.offset = (-1) * GPS.output * cos((GPS_bearing_difference / RAD_TO_DEGREES) + PI);
                roll.offset = GPS.output * sin((GPS_bearing_difference / RAD_TO_DEGREES) + PI);
                if(roll.offset > 18) roll.offset = 18; 
                else if(roll.offset < (-18)) roll.offset = -18;
                if(pitch.offset > 18) pitch.offset = 18; 
                else if(pitch.offset < (-18)) pitch.offset = -18;
            }
            p_loop_mode = loop_mode;
            
            //-----------------------------------------------------------------PID-------------------------------------------------------------------------------------
            
            /*------------
              Stab - PID
            ------------*/
            
            if(remote_y2 > 1 && !kill){
                roll.sum += (roll.error - roll.offset) * loop_time * 3;
                pitch.sum += (pitch.error - pitch.offset) * loop_time * 3;
                yaw.sum += (yaw_difference) * loop_time * 3;
            }
            
            roll.output = (roll.p * (roll.error - roll.offset) + roll.i * roll.sum);
            pitch.output = (pitch.p * (pitch.error - pitch.offset) + pitch.i * pitch.sum);
            yaw.output = (yaw.p * (yaw_difference) + yaw.i * yaw.sum);
            
            /*------------
              Rate - PID
            ------------*/
            
            roll_rate.error = gyro.y;   //roll.derivative;
            pitch_rate.error = gyro.x;  //pitch.derivative;
            yaw_rate.error = gyro.z;    //yaw.derivative;
            
            if(remote_y2 > 2 && !kill) {
                roll_rate.sum += (roll_rate.error + roll.output) * loop_time * 3;
                pitch_rate.sum += (pitch_rate.error + pitch.output) * loop_time * 3;
                yaw_rate.sum += (yaw_rate.error + yaw.output) * loop_time * 3;
            }
            
            roll_rate.output = (roll_rate.p * (roll_rate.error + roll.output) + roll_rate.i * roll_rate.sum);
            pitch_rate.output = (pitch_rate.p * (pitch_rate.error + pitch.output) + pitch_rate.i * pitch_rate.sum);
            if(remote_x2 < 3 && remote_x2 > (-3)) {
                yaw_rate.output = (yaw_rate.p * (yaw_rate.error + yaw.output) + yaw_rate.i * yaw_rate.sum);
            } else {
                yaw_rate.output = (yaw_rate.p * (yaw_rate.error + (float)remote_x2 / REMOTE_MAX * MaxYawRate));
                yaw.sum = 0;
                yaw.offset = yaw.error;                
            }
            
            //-------------------------------------------------------------Motor Output--------------------------------------------------------------------------------
            
            speed.upRight = altitude_rate.output - pitch_rate.output + roll_rate.output + yaw_rate.output;
            speed.downLeft = altitude_rate.output + pitch_rate.output - roll_rate.output + yaw_rate.output;
            speed.upLeft = altitude_rate.output - pitch_rate.output - roll_rate.output - yaw_rate.output; 
            speed.downRight = altitude_rate.output + pitch_rate.output + roll_rate.output - yaw_rate.output;
            
            LimitSpeed(&speed);
            
            //-----------------------------------------------------------Output to ESC's-------------------------------------------------------------------------------
            
            if(Xbee_signal && !kill) {
                WriteMotors(speed);
            } else {
                TurnMotorsOff();
            }
            
            //--------------------------------------------------------Send Data to remote-----------------------------------------------------------------------------
            
            if(tx_buffer_index++ == 24) {    
                SendFlightData(roll, pitch, yaw, altitude, loop_mode);
                tx_buffer_index = 0;
                tx_flag = 1;
            }
            
            //---------------------------------------------------------------------------------------------------------------------------------------------------------
            
            BMP_loop_timer++;
            
            while(loop_counter < 2040);                 //Loop should last at least 2.04 milliseconds
            loop_time = (double)loop_counter / 1000000.0; // Loop time in seconds:
        }
        //After killing and breaking from the main loop, stop all motors
        T4CONbits.ON = 0;//Stop the loop timer
        T6CONbits.ON = 0;
        TurnMotorsOff();
    }
}
