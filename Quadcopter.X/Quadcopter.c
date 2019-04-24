#include <xc.h>
#include "pragma.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <sys/attribs.h>

#include "settings.h"
#include "pic32.h"
#include "bitbang_I2C.h"
#include "PWM.h"
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
#include "ToF.h"
#include "EEPROM.h"   

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
#if board_version == 1
#warning "---------------------------------Building for board_v1!---------------------------------"
#elif board_version == 2
#warning "---------------------------------Building for board_v2!---------------------------------"
#elif board_version == 3
#warning "---------------------------------Building for board_v3!---------------------------------"
#elif board_version == 4
#warning "---------------------------------Building for board_v4!---------------------------------"
#endif

#define ACC_PURE_BUFFER_SIZE 50
#define TEST_FACTOR 10

void main() {    
    Motors speed;
    PID pitch, roll, yaw, altitude, GPS;
    XYZ acc_avg, acc_comp, acc_pure;                                                //Tilt-compensated acceleration
    float gravity_mag;
    int i;                                                                          //General purpose loop counter
    rx XBee_rx;
    float q[4];                                                                     //Quaternion
    float take_off_altitude, temperature;                                           //Offsets
    float heading, take_off_heading;                                                //yaw
    float remote_magnitude, remote_angle, remote_angle_difference;                  //RC
    float altitude_setpoint;                                                        //altitude
    int ToF_distance;                                                               //ToF data
    float latitude_offset, longitude_offset, take_off_latitude, take_off_longitude; //GPS
    float GPS_bearing_difference;                                                   //GPS bearing relative to yaw
    double loop_time;             
    char loop_mode, p_loop_mode;                                                    //Stabilize/alt-hold/pos-hold
    bool kill, p_kill;
    
    Init();
    
    WriteRGBLed(4095, 0, 0);                            //Red
    
    delay_ms(100);
    Init_10DOF();
    
    if(XBee.y2 > 29 && XBee.x2 > 13)                    //Calibrate ESCs
        CalibrateESC();
    
    TurnMotorsOff();
    
    delay_ms(100);
    if(XBee.y1 > 13 && XBee.x1 > 13) {                  //display sensor readings
        WriteRGBLed(4095, 0, 3800);                     //Magenta
        SendCalibrationData();
    }
    
    //Set PID gains
    SetPIDGain(&roll, &pitch, &yaw, &altitude, &GPS);
    
    delay_ms(1500);
    
    while(1) {
        ResetPID(&roll, &pitch, &yaw, &altitude, &GPS); //Clear PID variables
        ResetQuaternion(q);                             //Reset quaternion
        MotorsReset(&speed);                            //Clear motor speeds       
        
#if board_version == 4
        eeprom_readPID(&roll, &pitch, &yaw, &altitude, &GPS);
        eeprom_readCalibration();
#endif
        
        Menu(&roll, &pitch, &yaw, &altitude);
        
#if board_version == 4
        eeprom_writePID(&roll, &pitch, &yaw, &altitude, &GPS);
#endif
        
        WriteRGBLed(4095, 2500, 0); //Yellow
        
        delay_ms(100);
        
        DELAY_TIMER_ON = 1;
        //TX_TIMER_ON = 1;
        tx_buffer_index = 0;
        
        for(i = 0, VectorReset(&acc_avg), VectorReset(&gyro_avg); i < 1000; i++) {
            delay_counter = 0;
            GetAcc();
            GetRawGyro();
            GetCompass();
            
            acc_avg = VectorAdd(acc_avg, acc);
            gyro_avg = VectorAdd(gyro_avg, gyro);
            
            MadgwickQuaternionUpdate(q, acc, (XYZ){0.0, 0.0, 0.0}, compass, 0.050);
            
            XBeeWriteChar('B');
            XBeeWriteInt(i);
            XBeeWriteChar('\r');
            
            while(delay_counter < 3);
        }
        DELAY_TIMER_ON = 0;
        TX_TIMER_ON = 0;
        
        acc_avg = VectorScale(acc_avg, 1 / 1000.0);
        gyro_avg = VectorScale(gyro_avg, 1 / 1000.0);
        
        gravity_mag = sqrt(acc_avg.x * acc_avg.x + acc_avg.y * acc_avg.y + acc_avg.z * acc_avg.z);

        //Read initial heading
        take_off_heading = LimitAngle(-atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * RAD_TO_DEGREES - heading_offset);

        //Read take-off altitude
        altitude_KF_reset();
        take_off_altitude = GetTakeoffAltitude();
        
        if(GPS_signal) { 
            take_off_latitude = latitude; 
            take_off_longitude = longitude; 
        } else { 
            take_off_latitude = 0.0; 
            take_off_longitude = 0.0; 
        }
        
        loop_mode = 0;
        kill = 0;
        esc_counter = 0;
        data_aq_counter = 0;
        ToF_counter = 0;
        tx_buffer_timer = 0;
        altitude_setpoint = 0;
        
        LOOP_TIMER_ON = 1;
        //TX_TIMER_ON = 1;
        WriteRGBLed(0, 4095, 0);
        while(XBee.rs == 0) {
            while(tx_buffer_index)
                delay_ms(1);
            delay_ms(5);
            
            GetCompass(); 
            
            XBeeWriteChar('C');
            XBeeWriteFloat(compass_offset.x, 4);
            XBeeWriteChar(',');
            XBeeWriteFloat(compass_offset.y, 4);
            XBeeWriteChar(',');
            XBeeWriteFloat(compass_offset.z, 4);
            XBeeWriteChar(',');
            XBeeWriteFloat(compass.x, 4);
            XBeeWriteChar(',');
            XBeeWriteFloat(compass.y, 4);
            XBeeWriteChar(',');
            XBeeWriteFloat(compass.z, 4);
            XBeeWriteChar(',');
            XBeeWriteInt(loop_mode);
            XBeeWriteChar('\r');
        }
        
        XBee_rx = ReadXBee();
        
        //Main Loop
        while(XBee_rx.rs == 0) {
            
            //------------------------------------------------------------IMU data acquisition---------------------------------------------------------------------------
            if(data_aq_counter >= 50) {
                loop_time = (float)data_aq_counter / 1000000.0f;   // Loop time in seconds: 
                data_aq_counter = 0;                

                GetAcc();
                GetGyro();
                GetCompass(); 

                #if board_version == 4
                    if(ToF_counter >= 10) {
                        ToF_distance = ToF_readRange();
                        if(ToF_valueGood() != 0)
                            ToF_distance = -1;
                    }
                #endif

                //Update quaternion
                MadgwickQuaternionUpdate(q, acc, gyro, compass, loop_time);
                
                //Update altitude kalman filter
                GetCompensatedAcc(q, gravity_mag, &acc_pure, &acc_comp);
                altitude_KF_propagate(acc_comp.z, loop_time);         

                if(XBee_rx.y2 > 1 && !kill) {
                    PIDIntegrateAngle(&roll, loop_time);
                    PIDIntegrateAngle(&pitch, loop_time);
                    PIDIntegrateAngle(&yaw, loop_time);
                }
            }

            /*
            if(XBee.data_ready) {
                XBee_rx = ReadXBee();
                XBee_data_ready = 0;
            }
            */
            
            if(esc_counter >= ESC_TIME_us) {
                esc_counter = 0;
                p_kill = kill;
                p_loop_mode = loop_mode;
                XBee_rx = ReadXBee();

                //---------------------------------------------------------------Set Mode------------------------------------------------------------------------------

                if(XBee_rx.ls || !XBee_rx.signal) 
                    kill = 1; 
                else 
                    kill = 0;

                //Set loop mode - Stabilize, Alt-hold, Pos-hold
                if(XBee_rx.d1 == 0 || (XBee_rx.d1 == 2 && !GPS_signal)) loop_mode = 'S';
                else if(XBee_rx.d1 == 1) loop_mode = 'A';
                else if(XBee_rx.d1 == 2 && GPS_signal) loop_mode = 'P';

                if(p_loop_mode != loop_mode || p_kill != kill) {
                    if(kill) WriteRGBLed(4095, 4095, 4095);                 //White
                    else if(loop_mode == 'S') WriteRGBLed(0, 4095, 0);      //Green
                    else if(loop_mode == 'A') WriteRGBLed(0, 4095, 4095);   //Cyan
                    else if(loop_mode == 'P') WriteRGBLed(4095, 0, 4095);   //Magenta

                    if(loop_mode == 'A') {
                        altitude_setpoint = (float)XBee_rx.y2 / THROTTLE_MAX * MAX_SPEED;
                        altitude.sum = 0;
                        altitude.offset = altitude.error;
                    }
                    else if(loop_mode == 'P') {
                        latitude_offset = latitude;
                        longitude_offset = longitude;
                    }
                    yaw.offset = yaw.error;
                }                
                
                //---------------------------------------------------------------Altitude---------------------------------------------------------------------------------
                
                QuaternionToEuler(q, &roll.error, &pitch.error, &heading);
                
                yaw.error = LimitAngle(heading - take_off_heading);
                
                if(LoopAltitude(&altitude.error, &temperature)) {
                    altitude_KF_update(altitude.error);
                }
                altitude.error = altitude_KF_getAltitude() - take_off_altitude;
                altitude.derivative = -1.0 * altitude_KF_getVelocity();

                //------------------------------------------------Converting Remote data to a 2-D vector------------------------------------------------------------------

                if(loop_mode != 'P') {// If not in GPS mode
                    remote_magnitude = sqrt((float)XBee_rx.x1 * (float)XBee_rx.x1 + (float)XBee_rx.y1 * (float)XBee_rx.y1); //Magnitude of Remote's roll and pitch
                    if(XBee_rx.x1 == 0 && XBee_rx.y1 == 0) 
                        remote_angle = 0;
                    else 
                        remote_angle = -atan2((float)XBee_rx.x1, (float)XBee_rx.y1) * RAD_TO_DEGREES;  //Angle with respect to pilot/starting position
                    remote_angle_difference = LimitAngle(yaw.error - remote_angle); //Remote's angle with respect to quad's current direction

                    pitch.offset = max_pitch_roll_tilt * remote_magnitude / REMOTE_MAX * -cos(remote_angle_difference / RAD_TO_DEGREES);
                    roll.offset  = max_pitch_roll_tilt * remote_magnitude / REMOTE_MAX *  sin(remote_angle_difference / RAD_TO_DEGREES);
                }

                //---------------------------------------------------------------3 modes----------------------------------------------------------------------------------

                //--Stabilize--
                if(loop_mode == 'S') {
                    altitude.output = (float)XBee_rx.y2 / THROTTLE_MAX * MAX_SPEED;
                }

                //--Alt-hold---
                else if(loop_mode == 'A') {
                    if(XBee_rx.y2 > 10 && XBee_rx.y2 < 20) {  //Throttle stick in the mid position
                        altitude.sum += (altitude.offset - altitude.error) * loop_time;
                        altitude.output = (altitude.p * (altitude.offset - altitude.error) + altitude.i * altitude.sum + altitude.d * altitude.derivative) + altitude_setpoint;
                    } else {
                        if(XBee_rx.y2 <= 10) {
                            altitude.output = (altitude.d * (altitude.derivative - 3.0)) + altitude_setpoint;
                        }
                        else if(XBee_rx.y2 >= 20) {
                            altitude.output = (altitude.d * (altitude.derivative + 3.0)) + altitude_setpoint;
                        }
                        altitude.offset = altitude.error;                
                    }
                    //if(altitude_rate.output < 0.0) altitude_rate.output *= 1.2;
                }

                //--Pos-hold---
                else if(loop_mode == 'P') {
                    altitude.output = (float)XBee_rx.y2 / THROTTLE_MAX * MAX_SPEED;

                    GPS.error = DifferenceLatLon(take_off_latitude, take_off_longitude, latitude, longitude);
                    GPS_bearing_difference = LimitAngle(heading - DifferenceBearing(take_off_latitude, take_off_longitude, latitude, longitude));
                    GPS.output = (GPS.p * GPS.error); 

                    pitch.offset = -GPS.output * cos((GPS_bearing_difference / RAD_TO_DEGREES) + PI);
                    roll.offset = GPS.output * sin((GPS_bearing_difference / RAD_TO_DEGREES) + PI);

                    if(roll.offset > 18) 
                        roll.offset = 18; 
                    else if(roll.offset < (-18)) 
                        roll.offset = -18;
                    if(pitch.offset > 18) 
                        pitch.offset = 18; 
                    else if(pitch.offset < (-18)) 
                        pitch.offset = -18;
                }

                //---------------------------------------------------------Roll/Pitch/Yaw - PID----------------------------------------------------------------------------
                
                
                PIDDifferentiateAngle(&roll,  1.0f / ESC_FREQ);
                PIDDifferentiateAngle(&pitch, 1.0f / ESC_FREQ);
                PIDDifferentiateAngle(&yaw,   1.0f / ESC_FREQ);

                PIDOutputAngle(&roll);
                PIDOutputAngle(&pitch);
                
                    
                if(XBee_rx.x2 < 3 && XBee_rx.x2 > (-3)) {
                    PIDOutputAngle(&yaw);
                } else {
                    yaw.output = (yaw.d * (yaw.derivative + (float)XBee_rx.x2 / REMOTE_MAX * MAX_YAW_RATE));
                    yaw.sum = 0;
                    yaw.offset = yaw.error;                
                }

                //-------------------------------------------------------------Motor Output--------------------------------------------------------------------------------

                speed.upRight   = altitude.output - pitch.output + roll.output + (yaw.output * MOTOR_SPIN);
                speed.downLeft  = altitude.output + pitch.output - roll.output + (yaw.output * MOTOR_SPIN);
                speed.upLeft    = altitude.output - pitch.output - roll.output - (yaw.output * MOTOR_SPIN);
                speed.downRight = altitude.output + pitch.output + roll.output - (yaw.output * MOTOR_SPIN);

                LimitSpeed(&speed);

                //-----------------------------------------------------------Output to ESC's-------------------------------------------------------------------------------

                if(!kill)
                    WriteMotors(speed);
                else 
                    TurnMotorsOff();
            }
            
            //--------------------------------------------------------Send Data to remote-----------------------------------------------------------------------------
            if(tx_buffer_timer >= 50) {    
                //SendFlightData(roll, pitch, yaw, altitude, loop_mode);

                tx_buffer[0] = 'C';
                StrWriteFloat(compass.x * 10.0, 2, tx_buffer, 1);
                StrWriteFloat(compass.y * 10.0, 2, tx_buffer, 8);
                StrWriteFloat(compass.z * 10.0, 2, tx_buffer, 15);
                StrWriteFloat(altitude.offset - altitude.error, 2, tx_buffer, 22);
                StrWriteFloat(altitude.derivative, 8, tx_buffer, 29);
                StrWriteFloat(altitude.output - altitude_setpoint, 8, tx_buffer, 42);
                tx_buffer[55] = loop_mode;
                tx_buffer[56] = '\r';
                tx_buffer[57] = '\0';     

                tx_buffer_timer = 0;
                //tx_flag = 1;
            }
        }
        LOOP_TIMER_ON = 0;
        TX_TIMER_ON = 0;
        TurnMotorsOff();
    }
}