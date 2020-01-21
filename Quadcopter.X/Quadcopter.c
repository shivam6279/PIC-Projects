#include <xc.h>
#include "pragma.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "settings.h"
#include "pic32.h"
#include "bitbang_I2C.h"
#include "PWM.h"
#include "10DOF.h"
#include "3x3Matrix.h"
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

#define MODE_KILL 0
#define MODE_STABILIZE 1
#define MODE_ALT_HOLD 2
#define MODE_POS_HOLD 3

void main() {    
    Motors speed;
    PID pitch, roll, yaw, altitude, GPS;
    XYZ acc, gyro, compass;
    XYZ acc_comp;                                                                   //Tilt-compensated acceleration
    float gravity_mag;
    rx XBee_rx;
    float q[4];                                                                     //Quaternion
    float take_off_altitude, temperature;                                           //Offsets
    float baro_altitude, heading;                                                   //yaw
    float take_off_roll, take_off_pitch, take_off_heading;
    float remote_magnitude, remote_angle, remote_angle_difference;                  //RC
    float altitude_setpoint;                                                        //altitude
    int ToF_distance = -1;                                                          //ToF data
    float latitude_offset, longitude_offset, take_off_latitude, take_off_longitude; //GPS
    float GPS_bearing_difference;                                                   //GPS bearing relative to yaw
    double ESC_loop_time, gyro_loop_time, acc_loop_time, compass_loop_time;             
    char loop_mode, p_loop_mode;                                                    //Stabilize/alt-hold/pos-hold
    bool kill, p_kill;
    bool compute_acc_comp = false;
    
    //Startup initialization   
    Init();    
    
    WriteRGBLed(255, 0, 0);                            //Red
    
    delay_ms(200);
    
    bool EEPROM_connected = I2C_CheckAddress(EEPROM_ADDRESS);
    bool MPU6050_connected = I2C_CheckAddress(MPU6050_ADDR);
    bool compass_connected = I2C_CheckAddress(HMC5883_ADDR) | I2C_CheckAddress(QMC5883L_ADDR);
    bool ToF_connected = I2C_CheckAddress(VL6180X_ADDRESS);
    
    if(ToF_connected) 
        VL6180_init();
    
    Init_10DOF();    
    //Calibrate ESC
    if(XBee.y2 > 29 && XBee.x2 > 13) {
        CalibrateESC();
    }
    TurnMotorsOff();
    delay_ms(100);
    
    //Set PID gains
    SetPIDGain(&roll, &pitch, &yaw, &altitude, &GPS);
    delay_ms(1500);

    while(1) {
        ResetPID(&roll, &pitch, &yaw, &altitude, &GPS); //Clear PID variables
        ResetQuaternion(q);                             //Reset quaternion
        MotorsReset(&speed);        
                            //Clear motor speeds   
        
        if(EEPROM_connected) {
            if(!eeprom_readPID(&roll, &pitch, &yaw, &altitude, &GPS)) {
                SetPIDGain(&roll, &pitch, &yaw, &altitude, &GPS);
            }
            eeprom_readCalibration();
            eeprom_readOffsets();
        }
        
        // Wait to be armed
        Menu(&roll, &pitch, &yaw, &altitude);
        
        if(EEPROM_connected) {
            eeprom_writePID(&roll, &pitch, &yaw, &altitude, &GPS);
        }
        
        // Arm motors
        // Set up quaternion - save parameters at take off
        // Heading, altitude, latitude, longitude
        // Takes ~2 seconds
        ArmingSequence(q, &gravity_mag, &take_off_roll, &take_off_pitch, &take_off_heading, &take_off_altitude, &take_off_latitude, &take_off_longitude);
        
        loop_mode = MODE_KILL;
        kill = 0;
        altitude_setpoint = 0;
        ResetCounters();        
        LOOP_TIMER_ON = 1;
            
        XBee_rx = ReadXBee();
        
        //Main Loop
        while(XBee_rx.rs == 0) {
            
            //------------------------------------------------------------IMU data acquisition---------------------------------------------------------------------------
            if(gyro_aq_counter >= 125) {
                gyro_loop_time = (double)gyro_aq_counter / 1000000.0;    
                gyro_aq_counter = 0;
                
                GetGyro(&gyro);                
                MadgwickQuaternionUpdateGyro(q, gyro, gyro_loop_time);
                
                if(acc_aq_counter >= 1000) {
                    acc_loop_time = (double)acc_aq_counter / 1000000.0;
                    acc_aq_counter = 0;
                    
                    GetAcc(&acc);                    
                    MadgwickQuaternionUpdateAcc(q, acc, acc_loop_time);
                    
                    compute_acc_comp = true;
                    
                    if(ToF_connected) {
                        if(ToF_counter >= 50) {
                            ToF_counter = 0;
                            ToF_distance = ToF_readRange();
                            ToF_distance *= cos(TO_RAD(roll.error));
                            ToF_distance *= cos(TO_RAD(pitch.error));
                            if(ToF_valueGood() != 0)
                                ToF_distance = -1;
                        }
                    }                    
                    
                    if(compass_aq_counter >= 5000) {
                        compass_loop_time = (double)compass_aq_counter / 1000000.0;
                        compass_aq_counter = 0;
                        
                        GetCompass(&compass);
                        MadgwickQuaternionUpdate(q, acc, (XYZ){0, 0, 0}, compass, compass_loop_time);
                    }
                }
                
                QuaternionToEuler(q, &roll.error, &pitch.error, &heading);
                yaw.error = LimitAngle(heading - take_off_heading);
                
                roll.derivative  = gyro.x;
                pitch.derivative = gyro.y;
                yaw.derivative   = -gyro.z;
                
//                PIDDifferentiateAngle(&roll,  gyro_loop_time);
//                PIDDifferentiateAngle(&pitch, gyro_loop_time);
//                PIDDifferentiateAngle(&yaw,   gyro_loop_time);
                
                if(XBee_rx.y2 > MIN_THROTTLE_INTEGRATION && !kill) {
                    PIDIntegrateAngle(&roll,  gyro_loop_time);
                    PIDIntegrateAngle(&pitch, gyro_loop_time);
                    PIDIntegrateAngle(&yaw,   gyro_loop_time);
                }
                
                if(compute_acc_comp) {
                    compute_acc_comp = false;
                    
                    //Update altitude kalman filter
                    //acc_comp = RotateVectorEuler(acc, roll.error+roll_offset, pitch.error+pitch_offset, 0.0);
                    
                    if(fabs(roll.error) < 45.0 && fabs(pitch.error) < 45.0) {
                    
                        //acc_comp = MultiplyVectorQuaternion(acc, q);
                        //acc_comp.z -= gravity_mag;
                        
                        acc_comp = GetCompensatedAcc(q, acc, gravity_mag);
                        
                        if(fabs(acc_comp.z) < 10.0f) {
                            altitude_KF_propagate(acc_comp.z, acc_loop_time);                    
                        }
                    }
                    
                    if(fabs(altitude_KF_getVelocity()) > 10.0) {
                        altitude_KF_setVelocity(0.0);  
                    }
                }
            }

            //-------------------------------------------------------------Altitude acquisition--------------------------------------------------------------------------
            if(LoopAltitude(&baro_altitude, &temperature, true)) {
                if(fabs(baro_altitude - take_off_altitude) < 150.0) {
                    altitude_KF_update(baro_altitude);                
                    altitude.error = altitude_KF_getAltitude() - take_off_altitude;                

                    if(loop_mode == MODE_ALT_HOLD) {
                        if(XBee_rx.y2 > 10 && XBee_rx.y2 < 20)  //Throttle stick in the mid position
                            altitude.sum += (altitude.offset - altitude.error) * 0.002 * oversampling_delay;
                    }
                }
            }
            
            //-------------------------------------------------------------------RC input--------------------------------------------------------------------------------
            if(XBee.data_ready || !XBee.signal) {
                XBee_rx = ReadXBee();
                XBee.data_ready = 0;

                p_kill = kill;
                p_loop_mode = loop_mode;

                if(XBee_rx.ls || !XBee_rx.signal) 
                    kill = 1; 
                else 
                    kill = 0;

                //Set loop mode - Stabilize, Alt-hold, Pos-hold
                if(XBee_rx.d1 == 0 || (XBee_rx.d1 == 2 && !GPS_signal)) 
                    loop_mode = MODE_STABILIZE;
                else if(XBee_rx.d1 == 1) 
                    loop_mode = MODE_ALT_HOLD;
                else if(XBee_rx.d1 == 2 && GPS_signal) 
                    loop_mode = MODE_POS_HOLD;

                if(p_loop_mode != loop_mode || p_kill != kill) {
                    if(kill) 
                        WriteRGBLed(255, 255, 255);  //White
                    else if(loop_mode == MODE_STABILIZE) 
                        WriteRGBLed(0, 255, 0);        //Green
                    else if(loop_mode == MODE_ALT_HOLD) 
                        WriteRGBLed(0, 255, 255);     //Cyan
                    else if(loop_mode == MODE_POS_HOLD) 
                        WriteRGBLed(255, 0, 255);     //Magenta

                    if(loop_mode == MODE_ALT_HOLD) {
                        altitude_setpoint = (float)XBee_rx.y2 / THROTTLE_MAX * MAX_SPEED;
                        altitude.sum = 0;
                        altitude.offset = altitude.error;
                    }
                    else if(loop_mode == MODE_POS_HOLD) {
                        latitude_offset = latitude;
                        longitude_offset = longitude;
                    }
                    yaw.offset = yaw.error;
                }

                //Converting Remote data to a 2-D vector
                if(loop_mode != MODE_POS_HOLD) {// If not in GPS mode
                    remote_magnitude = sqrt((float)XBee_rx.x1 * (float)XBee_rx.x1 + (float)XBee_rx.y1 * (float)XBee_rx.y1); //Magnitude of Remote's roll and pitch
                    if(XBee_rx.x1 == 0 && XBee_rx.y1 == 0) 
                        remote_angle = 0;
                    else 
                        remote_angle = TO_DEG(-atan2((float)XBee_rx.x1, (float)XBee_rx.y1));                                //Angle with respect to pilot/starting position
                    remote_angle_difference = LimitAngle(yaw.error - remote_angle);                                         //Remote's angle with respect to quad's current direction

                    pitch.offset = max_pitch_roll_tilt * remote_magnitude / REMOTE_MAX * -cos(TO_RAD(remote_angle_difference));
                    roll.offset  = max_pitch_roll_tilt * remote_magnitude / REMOTE_MAX *  sin(TO_RAD(remote_angle_difference));
                }
            }

            //--------------------------------------------------------Send Data to remote-----------------------------------------------------------------------------
            if(TxBufferEmpty() && tx_buffer_timer > 50) {
                tx_buffer_timer = 0;
//                XBeePacketChar('C');
//                XBeePacketFloat(roll.error, 2); XBeePacketChar(',');
//                XBeePacketFloat(pitch.error, 2); XBeePacketChar(',');
//                XBeePacketFloat(yaw.error, 2); XBeePacketChar(',');
//                XBeePacketFloat(acc_comp.z, 2); XBeePacketChar(',');
//                XBeePacketFloat(altitude.sum, 8); XBeePacketChar(',');                      
//                XBeePacketFloat(altitude.output, 8); XBeePacketChar(',');
//                XBeePacketInt(loop_mode);
//                XBeePacketSend();
                
                XBeePacketChar('Z');
                XBeePacketStr("Pitch  : "); XBeePacketFixedFloat(pitch.error, 3, 2); XBeePacketChar('\n');
                XBeePacketStr("Roll   : "); XBeePacketFixedFloat(roll.error, 3, 2); XBeePacketChar('\n');
                XBeePacketStr("Heading: "); XBeePacketFixedFloat(heading, 3, 2); XBeePacketChar('\n');
                XBeePacketStr("Altitude: "); XBeePacketFloat(altitude.error, 2); XBeePacketChar('\n');
                XBeePacketStr("Altitude speed: "); XBeePacketFloat(altitude.derivative, 2); XBeePacketChar('\n');   
                
                XBeePacketStr("Acc X: "); XBeePacketFixedFloat(acc_comp.x, 3, 3); XBeePacketChar('\n');
                XBeePacketStr("Acc Y: "); XBeePacketFixedFloat(acc_comp.y, 3, 3); XBeePacketChar('\n');   
                XBeePacketStr("Acc Z: "); XBeePacketFixedFloat(acc_comp.z, 3, 3); XBeePacketChar('\n');
                
//                XBeePacketStr("Roll D : "); XBeePacketFixedFloat(roll.derivative, 3, 3); XBeePacketChar('\n');
//                XBeePacketStr("Pitch D: "); XBeePacketFixedFloat(pitch.derivative, 3, 3); XBeePacketChar('\n');   
//                XBeePacketStr("Yaw D  : "); XBeePacketFixedFloat(yaw.derivative, 3, 3); XBeePacketChar('\n');
                
                XBeePacketStr("ToF distance: "); XBeePacketFloat((float)ToF_distance / 10.0, 2); XBeePacketChar('\n');
                XBeePacketSend();
            }
            
            //--------------------------------------------------------PID Output to motors----------------------------------------------------------------------------
            if(esc_counter >= ESC_TIME_us) {
                ESC_loop_time = (float)esc_counter / 1000000.0f;                
                esc_counter = 0;     
                
                altitude.derivative = -1.0 * altitude_KF_getVelocity();

                //--Stabilize--
                if(loop_mode == MODE_STABILIZE) {
                    altitude.output = ((float)XBee_rx.y2 / THROTTLE_MAX * MAX_SPEED);
                }

                //--Alt-hold---
                else if(loop_mode == MODE_ALT_HOLD) {
                    
                    if(XBee_rx.y2 > 10 && XBee_rx.y2 < 20) {  //Throttle stick in the mid position
                        altitude.output = (altitude.p * (altitude.offset - altitude.error) + altitude.i * altitude.sum + altitude.d * altitude.derivative) + altitude_setpoint;
                    } else {
                        if(XBee_rx.y2 <= 10) {
                            altitude.output = (altitude.d * (altitude.derivative - max_altitude_rate)) + altitude.i * altitude.sum + altitude_setpoint;
                        }
                        else if(XBee_rx.y2 >= 20) {
                            altitude.output = (altitude.d * (altitude.derivative + max_altitude_rate)) + altitude.i * altitude.sum + altitude_setpoint;
                        }
                        altitude.offset = altitude.error;                
                    }
                    //if(altitude_rate.output < 0.0) altitude_rate.output *= 1.2;
                }

                //--Pos-hold---
                else if(loop_mode == MODE_POS_HOLD) {
                    altitude.output = (float)XBee_rx.y2 / THROTTLE_MAX * MAX_SPEED;

                    GPS.error = DifferenceLatLon(take_off_latitude, take_off_longitude, latitude, longitude);
                    GPS_bearing_difference = LimitAngle(heading - DifferenceBearing(take_off_latitude, take_off_longitude, latitude, longitude));
                    GPS.output = (GPS.p * GPS.error); 

                    pitch.offset = -GPS.output * cos(TO_RAD(GPS_bearing_difference) + PI);
                    roll.offset  =  GPS.output * sin(TO_RAD(GPS_bearing_difference) + PI);

                    roll.offset  = LimitValue(roll.offset, -max_pitch_roll_tilt, max_pitch_roll_tilt);
                    pitch.offset = LimitValue(pitch.offset, -max_pitch_roll_tilt, max_pitch_roll_tilt);
                }

                //Roll/Pitch/Yaw - PID     

                PIDOutputAngle(&roll);
                PIDOutputAngle(&pitch);
                    
                if(XBee_rx.x2 < 3 && XBee_rx.x2 > (-3)) {
                    PIDOutputAngle(&yaw);
                } else {
                    yaw.output = (yaw.d * (yaw.derivative + (float)XBee_rx.x2 / REMOTE_MAX * MAX_YAW_RATE));
                    yaw.sum = 0;
                    yaw.offset = yaw.error;                
                }
                
                if(roll.error < 30 && roll.error > -30)
                    altitude.output /= cos(TO_RAD(roll.error));
                if(pitch.error < 30 && pitch.error > -30)
                    altitude.output /= cos(TO_RAD(pitch.error));

                //Motor Output
                
                altitude.output = LimitValue(altitude.output, 0.0f, 900.0f);

                speed.upRight   = altitude.output - pitch.output + roll.output + (yaw.output * MOTOR_SPIN);
                speed.downLeft  = altitude.output + pitch.output - roll.output + (yaw.output * MOTOR_SPIN);
                speed.upLeft    = altitude.output - pitch.output - roll.output - (yaw.output * MOTOR_SPIN);
                speed.downRight = altitude.output + pitch.output + roll.output - (yaw.output * MOTOR_SPIN);

                LimitSpeed(&speed);

                //Output to ESC's

                if(!kill)
                    WriteMotors(speed);
                else 
                    TurnMotorsOff();
            }
        }
        TurnMotorsOff();
        
        //Finish previously started altitude measurement
        StartDelayCounter();
        while(!LoopAltitude(&altitude.error, &temperature, false) && ms_counter() < 100);
        StopDelayCounter();
        
        LOOP_TIMER_ON = 0;
        
    }
}