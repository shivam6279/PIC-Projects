#include <xc.h>
#include "pragma.h"
#include <math.h>
#include <stdbool.h>
#include <sys/attribs.h>
#include "settings.h"
#include "pic32.h"
#include "bitbang_I2C.h"
#include "PWMDriver.h"
#include "10DOF.h"
#include "altitude.h"
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
#warning "---------------------------------Building for Mini quad!--------------------------------"
#endif
#ifdef big
#warning "---------------------------------Building for Big quad!---------------------------------"
#endif

void main() {    
    Motors speed;
    PID pitch, roll, yaw, altitude, GPS;
    PID pitch_rate, roll_rate, yaw_rate, altitude_rate;
    XYZ acc_comp, acc_velocity, acc_position;                                       //Tilt-compensated acceleration
    int i;                                                                          //General purpose loop counter
    unsigned char tx_buffer_timer = 0;                                              //Counter to for the BMP delays
    float q[4];                                                                     //Quaternion
    double take_off_altitude, temperature;                                          //Offsets
    float heading, take_off_heading, yaw_difference;                                //yaw
    float remote_magnitude, remote_angle, remote_angle_difference;                  //RC
    double altitude_setpoint, acc_avg, altitude_buffer[ALTITUDE_BUFFER_SIZE];       //altitude
    double latitude_offset, longitude_offset, take_off_latitude, take_off_longitude;//GPS
    float GPS_bearing_difference;                                                   //GPS bearing relative to yaw
    double loop_time;             
    unsigned char altitude_stage;
    unsigned long int raw_temperature, raw_pressure;
    char loop_mode, p_loop_mode;                                                    //Stabilize/alt-hold/pos-hold
    bool kill, p_kill;
    
    Init();
    
    WriteRGBLed(4095, 0, 0);    //Red
    if(remote_y2 > 29 && remote_x2 > 13) { //Calibrate ESCs
        CalibrateESC();
    }
    TurnMotorsOff();
    
    if(remote_y1 > 13 && remote_x1 > 13) { //display sensor readings
        WriteRGBLed(4095, 0, 3800); //Purple
        DELAY_TIMER_ON = 1;
        TX_TIMER_ON = 1;
        tx_buffer_index = 0;
        while(1) {
            SendCalibrationData();
            tx_flag = 1;
            delay_counter = 0;
            while(delay_counter < 25);
        }
    }
    
    //Set PID gains
    SetPIDGain(&roll, &pitch, &yaw, &roll_rate, &pitch_rate, &yaw_rate, &altitude, &altitude_rate, &GPS);
    
    delay_ms(1500);
    
    while(1) {
        ResetPID(&roll, &pitch, &yaw, &roll_rate, &pitch_rate, &yaw_rate, &altitude, &altitude_rate, &GPS); //Clear PID variables
        ResetQuaternion(q);         //Reset quaternion
        MotorsReset(&speed);        //Clear motor speeds
        VectorReset(&acc_velocity);
        VectorReset(&acc_position);        
        //Menu
        menu(&roll, &pitch, &roll_rate, &pitch_rate, &altitude, &altitude_rate);
        
        WriteRGBLed(4095, 2500, 0); //Yellow
        
        delay_ms(100);
        CalibrateGyro();
        
        DELAY_TIMER_ON = 1;
        TX_TIMER_ON = 1;
        tx_buffer_index = 0;
        for(i = 0, acc_avg = 0, take_off_heading = 0; i < 1000; i++) {
            delay_counter = 0;
            GetAcc();
            GetGyro();
            GetCompass();
            acc_avg += sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z) / 1000.0;
            MadgwickQuaternionUpdate(q, acc, gyro, compass, 0.005);
            tx_buffer[0] = 'B';
            tx_buffer[1] = (i / 100 % 10) + 48;
            tx_buffer[2] = (i / 10 % 10) + 48;
            tx_buffer[3] = (i % 10) + 48;
            tx_buffer[4] = '\r';
            tx_buffer[5] = '\0';
            tx_flag = 1;
            while(delay_counter < 5);
        }
        DELAY_TIMER_ON = 0;
        TX_TIMER_ON = 0;
        //Read initial heading
        take_off_heading = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * RAD_TO_DEGREES - HEADINGOFFSET;
        LimitAngle(&take_off_heading);        
        //Read take-off altitude
        take_off_altitude = GetTakeoffAltitude(altitude_buffer);
        
        if(GPS_signal) { 
            take_off_latitude = latitude; 
            take_off_longitude = longitude; 
        } else { 
            take_off_latitude = 0.0; 
            take_off_longitude = 0.0; 
        }
        
        loop_mode = 0;
        kill = 0;
        altitude_stage = 0;
        tx_buffer_timer = 0;
        
        loop_time = 0.00204; 
        
        LOOP_TIMER_ON = 1;
        TX_TIMER_ON = 1;
        
        //Main Loop
        while(right_switch == 0) {
            loop_counter = 0;
            p_kill = kill;
            p_loop_mode = loop_mode;
            
            //---------------------------------------------------------------Set Mode------------------------------------------------------------------------------
            
            if(left_switch) kill = 1; 
            else kill = 0;
            //Set loop mode - Stabilize, Alt-hold, Pos-hold
            if(dial1 == 0 || (dial1 == 2 && !GPS_signal)) loop_mode = 'S';
            else if(dial1 == 1) loop_mode = 'A';
            else if(dial1 == 2 && GPS_signal) loop_mode = 'P';
            
            if(p_loop_mode != loop_mode || p_kill != kill) {
                if(kill) WriteRGBLed(4095, 4095, 4095);
                else if(loop_mode == 'S') WriteRGBLed(0, 4095, 0);   //Green
                else if(loop_mode == 'A') WriteRGBLed(0, 4095, 4095);//Cyan
                else if(loop_mode == 'P') WriteRGBLed(4095, 0, 4095);//Magenta
                
                if(loop_mode == 'A') {
                    altitude_setpoint = (float)remote_y2 * (MOTOR_MAX - MOTOR_OFF) / THROTTLE_MAX;
                    altitude.sum = 0;
                    altitude_rate.error = 0;
                    altitude.offset = altitude.error;
                }
                else if(loop_mode == 'P') {
                    latitude_offset = latitude;
                    longitude_offset = longitude;
                }
                yaw.offset = yaw.error;
            }
            
            //--------------------------------------------------------IMU data acquisition---------------------------------------------------------------------------
            
            GetAcc();
            GetGyro();
            GetCompass(); 
            LoopAltitude(&altitude_stage, &raw_pressure, &raw_temperature, altitude_buffer, &altitude, take_off_altitude, &temperature);
            
            //----------------------------------------------------------------Filters--------------------------------------------------------------------------------
             
            MadgwickQuaternionUpdate(q, acc, gyro, compass, loop_time); //Update the quaternion with the current accelerometer, gyroscope and magnetometer vectors
            MultiplyVectorQuarternion(q, acc, &acc_comp);
            
            QuaternionToEuler(q, &roll, &pitch, &yaw, &heading, &yaw_difference, take_off_heading);
            
            //------------------------------------------------Converting Remote data to a 2-D vector------------------------------------------------------------------
            
            if(loop_mode != 'P') {// If not in GPS mode
                remote_magnitude = sqrt((float)remote_x1 * (float)remote_x1 + (float)remote_y1 * (float)remote_y1); //Magnitude of Remote's roll and pitch
                if(remote_x1 == 0 && remote_y1 == 0) remote_angle = 0;
                else remote_angle = -atan2((float)remote_x1, (float)remote_y1) * RAD_TO_DEGREES;  //Angle with respect to pilot/starting position
                remote_angle_difference = yaw.error - remote_angle; //Remote's angle with respect to quad's current direction
                LimitAngle(&remote_angle_difference);
                pitch.offset = -MAX_PITCH_ROLL_TILT * remote_magnitude / REMOTE_MAX * cos(remote_angle_difference / RAD_TO_DEGREES);
                roll.offset = MAX_PITCH_ROLL_TILT * remote_magnitude / REMOTE_MAX * sin(remote_angle_difference / RAD_TO_DEGREES);
            }
            
            //---------------------------------------------------------------3 modes----------------------------------------------------------------------------------
            
            if(loop_mode == 'S') {
                altitude_rate.output = (float)remote_y2 * (MOTOR_MAX - MOTOR_OFF) / THROTTLE_MAX;
            }
            else if(loop_mode == 'A') {
                if(remote_y2 > 10 && remote_y2 < 20) {  //Throttle stick in the mid position
                    altitude.sum += (altitude.offset - altitude.error) * loop_time;
                    altitude.output = (altitude.p * (altitude.offset - altitude.error) + altitude.i * altitude.sum);
                    altitude_rate.output = (altitude_rate.p * (altitude_rate.error + altitude.output)) + altitude_setpoint;
                } else {
                    if(remote_y2 <= 10) {
                        altitude_rate.output = (altitude_rate.p * (altitude_rate.error - 3.0)) + altitude_setpoint;
                    }
                    else if(remote_y2 >= 20) {
                        altitude_rate.output = (altitude_rate.p * (altitude_rate.error + 3.0)) + altitude_setpoint;
                    }
                    altitude.offset = altitude.error;                
                }
                //if(altitude_rate.output < 0.0) altitude_rate.output *= 1.2;
                //altitude_rate.output += altitude_setpoint;
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
            
            //---------------------------------------------------------Roll/Pitch/Yaw - PID----------------------------------------------------------------------------
            
            /*------------
              Stab - PID
            ------------*/
            
            if(remote_y2 > 1 && !kill){
                roll.sum += (roll.error - roll.offset) * loop_time * 3.0;
                pitch.sum += (pitch.error - pitch.offset) * loop_time * 3.0;
                yaw.sum += (yaw_difference) * loop_time * 3.0;
            }
            
            roll.derivative = (roll.error - roll.p_error) / loop_time;
            pitch.derivative = (pitch.error - pitch.p_error) / loop_time;
            yaw.derivative = (yaw.error - yaw.p_error) / loop_time;
            
            roll.output = (roll.p * (roll.error - roll.offset) + roll.i * roll.sum);
            pitch.output = (pitch.p * (pitch.error - pitch.offset) + pitch.i * pitch.sum);
            yaw.output = (yaw.p * (yaw_difference) + yaw.i * yaw.sum);
            
            /*------------
              Rate - PID
            ------------*/
            
            roll_rate.error = roll.derivative;  //gyro.y
            pitch_rate.error = pitch.derivative;//gyro.x
            yaw_rate.error = yaw.derivative;    //gyro.z
            
            if(remote_y2 > 2 && !kill) {
                roll_rate.sum += (roll_rate.error + roll.output) * loop_time * 3;
                pitch_rate.sum += (pitch_rate.error + pitch.output) * loop_time * 3;
                yaw_rate.sum += (yaw_rate.error + yaw.output) * loop_time * 3;
            }
            
            roll_rate.output = (roll_rate.p * (roll_rate.error + roll.output));
            pitch_rate.output = (pitch_rate.p * (pitch_rate.error + pitch.output));
            if(remote_x2 < 3 && remote_x2 > (-3)) {
                yaw_rate.output = (yaw_rate.p * (yaw_rate.error + yaw.output) + yaw_rate.i * yaw_rate.sum);
            } else {
                yaw_rate.output = (yaw_rate.p * (yaw_rate.error + (float)remote_x2 / REMOTE_MAX * MAX_YAW_RATE));
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
            
            if(tx_buffer_timer++ >= 10) {    
                SendFlightData(roll, pitch, yaw, altitude, loop_mode);
                tx_buffer_timer = 0;
                tx_flag = 1;
            }
            
            //---------------------------------------------------------------------------------------------------------------------------------------------------------
            
            while(loop_counter < 2040);                     //Loop should last at least 2.04 milliseconds
            loop_time = (double)loop_counter / 1000000.0;   // Loop time in seconds:
        }
        LOOP_TIMER_ON = 0;
        TX_TIMER_ON = 0;
        TurnMotorsOff();
    }
}
