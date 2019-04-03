#include <xc.h>
#include "pragma.h"
#include <math.h>
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

#if board_version == 4
    #include "EEPROM.h"
    
#endif

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
    unsigned char tx_buffer_timer = 0;                                              //Counter to for the BMP delays
    float q[4];                                                                     //Quaternion
    double take_off_altitude, temperature;                                          //Offsets
    float heading, take_off_heading, yaw_difference;                                //yaw
    float remote_magnitude, remote_angle, remote_angle_difference;                  //RC
    double altitude_setpoint, altitude_buffer[ALTITUDE_BUFFER_SIZE];                //altitude
    int ToF_distance;                                                               //ToF data
    double latitude_offset, longitude_offset, take_off_latitude, take_off_longitude;//GPS
    float GPS_bearing_difference;                                                   //GPS bearing relative to yaw
    double loop_time;             
    unsigned char altitude_stage;
    unsigned long int raw_temperature, raw_pressure;
    char loop_mode, p_loop_mode;                                                    //Stabilize/alt-hold/pos-hold
    bool kill, p_kill;
    
    Init();
    delay_ms(100);
    Init_10DOF();
    
    if(XBee.y2 > 29 && XBee.x2 > 13) {                  //Calibrate ESCs
        WriteRGBLed(0, 4095, 4095);   
        delay_ms(5000);                                 //Cyan
        CalibrateESC();
    }
    
    WriteRGBLed(4095, 0, 0);                            //Red
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
        
        Menu(&roll, &pitch, &altitude);
        
        WriteRGBLed(4095, 2500, 0); //Yellow
        
        delay_ms(100);
        CalibrateGyro();
        
        DELAY_TIMER_ON = 1;
        TX_TIMER_ON = 1;
        tx_buffer_index = 0;
             
        for(i = 0, VectorReset(&acc_avg), VectorReset(&gyro_avg); i < 1000; i++) {
            delay_counter = 0;
            GetAcc();
            GetRawGyro();
            GetCompass();
            
            acc_avg = VectorAdd(acc_avg, acc);
            gyro_avg = VectorAdd(gyro_avg, gyro);
            
            MadgwickQuaternionUpdate(q, acc, (XYZ){0.0, 0.0, 0.0}, compass, 0.050);
            
            tx_buffer[0] = 'B';
            tx_buffer[1] = (i / 100 % 10) + 48;
            tx_buffer[2] = (i / 10 % 10) + 48;
            tx_buffer[3] = (i % 10) + 48;
            tx_buffer[4] = '\r';
            tx_buffer[5] = '\0';
            tx_flag = 1;
            
            while(delay_counter < 3);
        }
        DELAY_TIMER_ON = 0;
        TX_TIMER_ON = 0;
        
        acc_avg = VectorScale(acc_avg, 1 / 1000.0);
        gyro_avg = VectorScale(gyro_avg, 1 / 1000.0);
        
        gravity_mag = sqrt(acc_avg.x * acc_avg.x + acc_avg.y * acc_avg.y + acc_avg.z * acc_avg.z);
        //MultiplyVectorQuarternion(q, acc, &gravity);

        //Read initial heading
        take_off_heading = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * RAD_TO_DEGREES - HEADINGOFFSET;
        LimitAngle(&take_off_heading);   

        //Read take-off altitude
        altitude_KF_reset();
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
        ToF_counter = 0;
        tx_buffer_timer = 0;
        
        LOOP_TIMER_ON = 1;
        TX_TIMER_ON = 1;
        
        //Main Loop
        while(XBee.rs == 0) {
            loop_time = (double)loop_counter / 1000000.0;   // Loop time in seconds: 
            loop_counter = 0;
            
            //--------------------------------------------------------IMU data acquisition---------------------------------------------------------------------------

            GetAcc();
            GetGyro();
            GetCompass(); 
            
            #if board_version == 4
                if(ToF_counter >= 10) {
                    ToF_distance = ToF_readRange();
                    if(ToF_valueGood != 0)
                        ToF_distance = -1;
                }
            #endif

            //------------------------------------------------------Update quaternion and get angles-----------------------------------------------------------------

            MadgwickQuaternionUpdate(q, acc, gyro, compass, loop_time); //Update the quaternion with the current accelerometer, gyroscope and magnetometer vectors
            GetCompensatedAcc(q, gravity_mag, &acc_pure, &acc_comp);
            altitude_KF_propagate(acc_comp.z, loop_time);         

            if(XBee_rx.y2 > 1 && !kill){
                roll.sum  += (roll.error  - roll.offset)  * loop_time;
                pitch.sum += (pitch.error - pitch.offset) * loop_time;
                yaw.sum   += (yaw_difference)             * loop_time;
            }
          
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
                        altitude.derivative = 0;
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

                yaw.error = heading - take_off_heading;
                LimitAngle(&yaw.error);
                yaw_difference = yaw.error - yaw.offset;
                LimitAngle(&yaw_difference);
                
                LoopAltitude(&altitude_stage, &raw_pressure, &raw_temperature, altitude_buffer, take_off_altitude, &temperature);
                altitude.error = altitude_KF_getAltitude() - take_off_altitude;
                altitude.derivative = -1.0 * altitude_KF_getVelocity();

                //------------------------------------------------Converting Remote data to a 2-D vector------------------------------------------------------------------

                if(loop_mode != 'P') {// If not in GPS mode
                    remote_magnitude = sqrt((float)XBee_rx.x1 * (float)XBee_rx.x1 + (float)XBee_rx.y1 * (float)XBee_rx.y1); //Magnitude of Remote's roll and pitch
                    if(XBee_rx.x1 == 0 && XBee_rx.y1 == 0) 
                        remote_angle = 0;
                    else 
                        remote_angle = -atan2((float)XBee_rx.x1, (float)XBee_rx.y1) * RAD_TO_DEGREES;  //Angle with respect to pilot/starting position
                    remote_angle_difference = yaw.error - remote_angle; //Remote's angle with respect to quad's current direction
                    LimitAngle(&remote_angle_difference);

                    pitch.offset = -MAX_PITCH_ROLL_TILT * remote_magnitude / REMOTE_MAX * cos(remote_angle_difference / RAD_TO_DEGREES);
                    roll.offset = MAX_PITCH_ROLL_TILT * remote_magnitude / REMOTE_MAX * sin(remote_angle_difference / RAD_TO_DEGREES);
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
                    //altitude_rate.output += altitude_setpoint;
                }

                //--Pos-hold---
                else if(loop_mode == 'P') {
                    altitude.output = (float)XBee_rx.y2 / THROTTLE_MAX * MAX_SPEED;
                    GPS.error = DifferenceLatLon(take_off_latitude, take_off_longitude, latitude, longitude);
                    GPS_bearing_difference = heading - DifferenceBearing(take_off_latitude, take_off_longitude, latitude, longitude);
                    LimitAngle(&GPS_bearing_difference);
                    GPS.output = (GPS.p * GPS.error); 
                    pitch.offset = -1 * GPS.output * cos((GPS_bearing_difference / RAD_TO_DEGREES) + PI);
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
                
                roll.derivative  = (roll.error  - roll.p_error)  * ESC_FREQ;
                pitch.derivative = (pitch.error - pitch.p_error) * ESC_FREQ;
                yaw.derivative   = (yaw.error   - yaw.p_error)   * ESC_FREQ;
                
                roll.p_error = roll.error;
                pitch.p_error = pitch.error;
                yaw.p_error = yaw.error;

                roll.output  = (roll.p  * (roll.error  - roll.offset)  + roll.i  * roll.sum  + roll.d  * roll.derivative);
                pitch.output = (pitch.p * (pitch.error - pitch.offset) + pitch.i * pitch.sum + pitch.d * pitch.derivative);

                if(XBee_rx.x2 < 3 && XBee_rx.x2 > (-3)) {
                    yaw.output = (yaw.p * (yaw_difference) + yaw.i * yaw.sum + yaw.d * yaw.derivative);
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

                if(!kill) {
                    WriteMotors(speed);
                } else {
                    TurnMotorsOff();
                }

                //--------------------------------------------------------Send Data to remote-----------------------------------------------------------------------------

                if(tx_buffer_timer++ >= 10) {    
                    //SendFlightData(roll, pitch, yaw, altitude, loop_mode);

                    tx_buffer[0] = 'C';
                    StrWriteFloat(roll.error, 3, 2, tx_buffer, 1);
                    StrWriteFloat(pitch.error, 3, 2, tx_buffer, 8);
                    StrWriteFloat(yaw.error, 3, 2, tx_buffer, 15);
                    StrWriteFloat(altitude.offset - altitude.error, 3, 2, tx_buffer, 22);
                    StrWriteFloat(altitude.derivative, 3, 8, tx_buffer, 29);
                    StrWriteFloat(altitude.output - altitude_setpoint, 3, 8, tx_buffer, 42);
                    tx_buffer[55] = loop_mode;
                    tx_buffer[56] = '\r';
                    tx_buffer[57] = '\0';     

                    tx_buffer_timer = 0;
                    tx_flag = 1;
                }

                //---------------------------------------------------------------------------------------------------------------------------------------------------------                
            }
        }
        LOOP_TIMER_ON = 0;
        TX_TIMER_ON = 0;
        TurnMotorsOff();
    }
}