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
#include "MPU6050.h"
#include "LIS3MDL.h"
#include "QMC5883.h"
#include "BMP390.h"
#include "3x3Matrix.h"
#include "altitude.h"
#include "USART.h"
#include "SPI.h"
#include "XBee.h"
#include "PID.h"
#include "motor.h"
#include "GPS.h"
#include "AHRS.h"
#include "init.h"
#include "menu.h"
#include "ToF.h"
#include "TF_Luna.h"
#include "EEPROM.h"

#define MODE_KILL 0
#define MODE_STABILIZE 1
#define MODE_ALT_HOLD 2
#define MODE_POS_HOLD 3
#define MODE_ACRO 4

int main() {    
    Motors speed;
    PID pitch, roll, yaw;
    XYZ acc = {0.0, 0.0, 0.0}, gyro = {0.0, 0.0, 0.0}, compass = {0.0, 0.0, 0.0};
    XYZ acc_comp;
    float gravity_mag;
    rx XBee_rx;
    float q[4] = {1.0,0.0,0.0,0.0};
    float heading;
    float take_off_roll, take_off_pitch, take_off_heading;
    float remote_magnitude, remote_angle, remote_angle_difference;
    
    // Altitude
    PID altitude;
    float altitude_setpoint, baro_altitude = 0.0;
    float takeoff_altitude, temperature;
    
    // ToF
    PID lidar;
    float lidar_dist = -1;
    float takeoff_lidar_dist = 0; 
    int TF_luna_dist, TF_flux;
    int takeoff_TF_luna_dist = 0;
    
    // GPS
    PID gps;
    float latitude_offset, longitude_offset, take_off_latitude, take_off_longitude;
    float gps_bearing_difference;
    
    
    double ESC_loop_time, gyro_loop_time, acc_loop_time, compass_loop_time;             
    char loop_mode, p_loop_mode; //Stabilize/alt-hold/pos-hold
    bool kill, p_kill;
    bool compute_acc_comp = false;
    
    int i, j, k;
    
    //Startup initialization   
    Init();    
    
    for(i = 0; i < 40; i++) {
        WriteRGBLed(255, 0, 0);  //Red
        delay_ms(5);
    }
    
    bool EEPROM_connected = I2C_CheckAddress(EEPROM_ADDRESS);
    bool MPU6050_connected = I2C_CheckAddress(MPU6050_ADDR);
    bool compass_connected = I2C_CheckAddress(LIS3MDL_ADDR);
    bool ToF_connected = I2C_CheckAddress(VL6180X_ADDRESS);
    
    delay_ms(100);
     
    Init10DOF();
    VL53L0X_init();
    VL53L0X_startContinuous(0);
    
    //Calibrate ESC
    if(XBee.y2 > 29 && XBee.x2 > 13) {
        CalibrateESC();
    }
    
    TurnMotorsOff();
    delay_ms(100);
    
    //Set PID gains
    PIDSet(&lidar, 0.5f, 0.15f, 20.0f);
    PIDSetIntegralParams(&lidar, 850.0f, 250.0f);
    PIDSetDiff(&lidar, -1);
    
    SetPIDGain(&roll, &pitch, &yaw, &altitude, &gps);
    PIDSetIntegralParams(&roll,  ANTI_WINDUP_MAX_BOUND, ANTI_WINDUP_MAX_ANGLE);
    PIDSetIntegralParams(&pitch, ANTI_WINDUP_MAX_BOUND, ANTI_WINDUP_MAX_ANGLE);
    delay_ms(1500);
    
    WriteRGBLed(0, 0, 0);
    
    GPS_TIMER_ON = 1;
    
    bool i2c_addr_list[128];
    I2C_device_list(i2c_addr_list);
    
    USART4_send_str("I2C devices\n");
    for(i = 1; i < 128; i++) {
        if(i2c_addr_list[i]) {
            USART4_write_int_hex(i);
            USART4_send_str("\n");
        }
    }
    delay_ms(1000);
    
    /*while(1) {
        StartDelayCounter();
                
        GetAcc(&acc);
        GetGyro(&gyro);
        GetCompass(&compass);
        
        heading = TO_DEG(atan2(compass.y, compass.x));        
        MadgwickQuaternionUpdate(q, acc, gyro, compass, 0.025);
//        MadgwickQuaternionUpdateGyro(q, gyro, 0.025);
        QuaternionToEuler(q, &roll.error, &pitch.error, &yaw.error);
        
        USART4_write_float(pitch.error, 2);        
        USART4_send_str(", ");
        USART4_write_float(roll.error, 2);        
        USART4_send_str(", ");
        USART4_write_float(yaw.error, 2);
        USART4_send('\n');
        
        while(ms_counter() < 25);
    }*/
    
    /*while(1) {
        unsigned long temp, pressure;
        temp = BMP390_compensate_temp(BMP390_read_temp());
        pressure = BMP390_compensate_pressure(BMP390_read_pressure(), temp);
        altitude.error = pressure_to_altitude(pressure);
        
        USART4_write_float(altitude.error, 2);        
        USART4_send_str(", ");
        USART4_write_float((float)pressure/100.0f, 2);        
        USART4_send_str(", ");
        USART4_write_float(temp, 2);
        USART4_send('\n');
        
        delay_ms(100);
    }*/
    
    while(1) {
        //Clear PID variables
        PIDReset(&roll);
        PIDReset(&pitch);
        PIDReset(&yaw);
        PIDReset(&altitude);
        PIDReset(&gps);
        PIDReset(&lidar);
        ResetQuaternion(q);                             //Reset quaternion
        MotorsReset(&speed);                            //Clear motor speeds   
        
#if USE_EEPROM
        if(EEPROM_connected) {
            if(!eeprom_readPID(&roll, &pitch, &yaw, &altitude, &GPS)) {
                SetPIDGain(&roll, &pitch, &yaw, &altitude, &GPS);
            }
            eeprom_readCalibration();
            eeprom_readOffsets();
        }
#endif  
        
        // Wait to be armed
        Menu(&roll, &pitch, &yaw, &altitude);

#if USE_EEPROM   
        if(EEPROM_connected) {
            eeprom_writePID(&roll, &pitch, &yaw, &altitude, &GPS);
        }
#endif
        // Arm motors
        // Set up quaternion - save parameters at take off
        // Heading, altitude, latitude, longitude
        // Takes ~2 seconds
        ArmingSequence(q, &gravity_mag, &take_off_roll, &take_off_pitch, &take_off_heading, &takeoff_altitude, &take_off_latitude, &take_off_longitude);
        
        takeoff_lidar_dist = VL53L0X_readRange();
        lidar_dist = takeoff_lidar_dist;
        TF_luna_getData(&takeoff_TF_luna_dist, &TF_flux);
        TF_luna_dist = takeoff_TF_luna_dist;
        
        loop_mode = MODE_KILL;
        kill = 0;
        altitude_setpoint = 0;
        ResetCounters();        
        LOOP_TIMER_ON = 1;        
        
        XBee_rx = ReadXBee();
        
        lidar.offset = 150.0f;
        
        //Main Loop
        while(XBee_rx.rs == 0) {
            
            //------------------------------------------------------------IMU data acquisition---------------------------------------------------------------------------
            if(gyro_aq_counter >= 2500) {
                gyro_loop_time = (double)gyro_aq_counter / 1000000.0;    
                gyro_aq_counter = 0;
                
                GetGyro(&gyro);                
                MadgwickQuaternionUpdateGyro(q, gyro, gyro_loop_time);
                
                if(acc_aq_counter >= 2500) {
                    acc_loop_time = (double)acc_aq_counter / 1000000.0;
                    acc_aq_counter = 0;
                    
                    GetAcc(&acc);                    
                    MadgwickQuaternionUpdateAcc(q, acc, acc_loop_time);
                    
                    compute_acc_comp = true;
                    
                    if(compass_aq_counter >= 5000) {
                        compass_loop_time = (double)compass_aq_counter / 1000000.0;
                        compass_aq_counter = 0;
                        
                        GetCompass(&compass);
                        MadgwickQuaternionUpdate(q, acc, (XYZ){0, 0, 0}, compass, compass_loop_time);
                    }
                }
                
                QuaternionToEuler(q, &roll.error, &pitch.error, &heading);
                yaw.error = LimitAngle(heading - take_off_heading);
                
                roll.derivative  = gyro.y;
                pitch.derivative = gyro.x;
                yaw.derivative   = gyro.z;
                
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
                    
                    if(fabs(roll.error) < 45.0 && fabs(pitch.error) < 45.0) {
                    
                        //acc_comp = MultiplyVectorQuaternion(acc, q);
                        //acc_comp.z -= gravity_mag;
                        
                        acc_comp = GetCompensatedAcc(q, acc, gravity_mag);
                        
//                        if(fabs(acc_comp.z) < 10.0f) {
//                            altitude_KF_propagate(acc_comp.z, acc_loop_time);                    
//                        }
                    }
                    
                    if(fabs(altitude_KF_getVelocity()) > 10.0) {
                        altitude_KF_setVelocity(0.0);  
                    }
                }
            }

            //-------------------------------------------------------------Altitude acquisition--------------------------------------------------------------------------
            if(baro_aq_counter >= 250) {
                baro_altitude = GetAltitude();
                altitude.error = baro_altitude - takeoff_altitude;
                /*if(fabs(baro_altitude - takeoff_altitude) < 150.0) {
                    altitude_KF_update(baro_altitude);                
                    altitude.error = altitude_KF_getAltitude() - takeoff_altitude;                

                    if(loop_mode == MODE_ALT_HOLD) {
                        if(XBee_rx.y2 > 10 && XBee_rx.y2 < 20)  //Throttle stick in the mid position
                            altitude.integral += (altitude.offset - altitude.error) * 0.002 * oversampling_delay;
                    }
                }*/
            }
            
            if(ToF_counter >= 25) {
                int lidar_temp;
                ToF_counter = 0;
                TF_luna_getData(&lidar_temp, &TF_flux);
                if(lidar_temp != 0) {
                    TF_luna_dist = 0.5 * (float)lidar_temp + 0.5 * TF_luna_dist;
                }

                VL53L0X_readRange();
                
                lidar.p_error = lidar.error;
                lidar.error = 0.3 * (VL53L0X_bufferAvg() - takeoff_lidar_dist) + 0.7 * lidar.error;
                PIDIntegrate(&lidar, 0.025f);
                lidar.derivative = 0.1 * (VL53L0X_bufferDelta() / 0.025f) + 0.9 * lidar.derivative;
//                lidar.derivative = (lidar.p_error - lidar.error) / 0.025f;
//                PIDOutput(&lidar);
                
                /*if(ToF_connected) {
                    if(ToF_counter >= 50) {
                        ToF_counter = 0;
                        ToF_distance = ToF_readRange();
                        ToF_distance *= cos(TO_RAD(roll.error));
                        ToF_distance *= cos(TO_RAD(pitch.error));
                        if(ToF_valueGood() != 0)
                            ToF_distance = -1;
                    }
                }*/ 
            }
            
            //-------------------------------------------------------------------RC input--------------------------------------------------------------------------------
            if(XBee.data_ready || !XBee.signal) {
                XBee_rx = ReadXBee();
                XBee.data_ready = 0;

                p_kill = kill;
                p_loop_mode = loop_mode;

                if(XBee_rx.ls || !XBee_rx.signal) {
                    kill = 1; 
                } else {
                    kill = 0;
                }
                //Set loop mode - Stabilize, Alt-hold, Pos-hold
                if(XBee_rx.d1 == 0 || (XBee_rx.d1 == 2 && !gps_signal)) {
                    loop_mode = MODE_STABILIZE;
                } else if(XBee_rx.d1 == 1) { 
                    loop_mode = MODE_ALT_HOLD;
                } else if(XBee_rx.d1 == 2 && gps_signal) { 
                    loop_mode = MODE_POS_HOLD;
                }

                if(p_loop_mode != loop_mode || p_kill != kill) {
                    if(kill) {
                        WriteRGBLed(255, 255, 255);  //White
                    } else if(loop_mode == MODE_STABILIZE) {
                        WriteRGBLed(0, 255, 0);        //Green
                    } else if(loop_mode == MODE_ALT_HOLD) {
                        WriteRGBLed(0, 255, 255);     //Cyan
                    } else if(loop_mode == MODE_POS_HOLD) { 
                        WriteRGBLed(255, 0, 255);     //Magenta
                    }
        
                    if(loop_mode == MODE_ALT_HOLD) {
//                        altitude_setpoint = (float)XBee_rx.y2 / THROTTLE_MAX * MAX_SPEED;
//                        altitude.integral = 0;
//                        altitude.offset = altitude.error;
                        lidar.output = 0.0f; 
                        lidar.offset = lidar.error;
                        lidar.integral = ((float)XBee_rx.y2 / THROTTLE_MAX * MAX_SPEED);
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
                    if(XBee_rx.x1 == 0 && XBee_rx.y1 == 0) {
                        remote_angle = 0;
                    } else { 
                        remote_angle = TO_DEG(-atan2((float)XBee_rx.x1, (float)XBee_rx.y1));                                //Angle with respect to pilot/starting position
                    }
                    remote_angle_difference = LimitAngle(yaw.error - remote_angle);                                         //Remote's angle with respect to quad's current direction
//                    remote_angle_difference = LimitAngle(remote_angle); 
                    pitch.offset = max_pitch_roll_tilt * remote_magnitude / REMOTE_MAX * -cos(TO_RAD(remote_angle_difference));
                    roll.offset  = max_pitch_roll_tilt * remote_magnitude / REMOTE_MAX * sin(TO_RAD(remote_angle_difference));
                }
            }

            //--------------------------------------------------------Send Data to remote-----------------------------------------------------------------------------
            if(TxBufferEmpty() && tx_buffer_timer > 50) {
                tx_buffer_timer = 0;
                XBeePacketChar('C');
                XBeePacketFloat(pitch.error, 2); XBeePacketChar(',');       // Pitch
                XBeePacketFloat(roll.error, 2); XBeePacketChar(',');        // Roll
                XBeePacketFloat(yaw.error, 2); XBeePacketChar(',');         // Yaw
                XBeePacketFloat(TO_DEG(atan2(compass.y, compass.x)), 2); XBeePacketChar(',');     // Altitude
                XBeePacketFloat(latitude, 6); XBeePacketChar(',');          // Latitude
                XBeePacketFloat(longitude, 6); XBeePacketChar(',');         // Longitude
                XBeePacketInt(loop_mode * !kill);
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
                    
                    /*if(XBee_rx.y2 > 10 && XBee_rx.y2 < 20) {  //Throttle stick in the mid position
                        altitude.output = (altitude.kp * (altitude.offset - altitude.error) + altitude.ki * altitude.integral + altitude.kd * altitude.derivative) + altitude_setpoint;
                    } else {
                        if(XBee_rx.y2 <= 10) {
                            altitude.output = (altitude.kd * (altitude.derivative - max_altitude_rate)) + altitude.ki * altitude.integral + altitude_setpoint;
                        }
                        else if(XBee_rx.y2 >= 20) {
                            altitude.output = (altitude.kd * (altitude.derivative + max_altitude_rate)) + altitude.ki * altitude.integral + altitude_setpoint;
                        }
                        altitude.offset = altitude.error;                
                    }
                    //if(altitude_rate.output < 0.0) altitude_rate.output *= 1.2;*/
                    PIDOutput(&lidar);
                }

                //--Pos-hold---
                else if(loop_mode == MODE_POS_HOLD) {
                    altitude.output = (float)XBee_rx.y2 / THROTTLE_MAX * MAX_SPEED;

                    gps.error = DifferenceLatLon(take_off_latitude, take_off_longitude, latitude, longitude);
                    gps_bearing_difference = LimitAngle(heading - DifferenceBearing(take_off_latitude, take_off_longitude, latitude, longitude));
                    gps.output = (gps.kp * gps.error); 

                    pitch.offset = -gps.output * cos(TO_RAD(gps_bearing_difference) + PI);
                    roll.offset  =  gps.output * sin(TO_RAD(gps_bearing_difference) + PI);

                    roll.offset  = LimitValue(roll.offset, -max_pitch_roll_tilt, max_pitch_roll_tilt);
                    pitch.offset = LimitValue(pitch.offset, -max_pitch_roll_tilt, max_pitch_roll_tilt);
                }

                //Roll/Pitch/Yaw - PID     
                
                roll.output = roll.kp * LimitAngle(roll.error - roll.offset) + roll.ki * roll.integral + roll.kd * roll.derivative;
                pitch.output = pitch.kp * LimitAngle(pitch.error - pitch.offset) + pitch.ki * pitch.integral + pitch.kd * pitch.derivative;                
//                roll.output = roll.kd * (roll.derivative - roll.offset*5.0);
//                pitch.output = pitch.kd * (pitch.derivative - pitch.offset*5.0);
                    
                if(XBee_rx.x2 == 0) {
                    yaw.output = yaw.kp * LimitAngle(yaw.error - yaw.offset) + yaw.ki * yaw.integral + yaw.kd * yaw.derivative;   
//                    yaw.output = yaw.kd * yaw.derivative;
                } else {
                    yaw.output = (yaw.kd * (yaw.derivative + (float)XBee_rx.x2 / REMOTE_MAX * MAX_YAW_RATE));
                    yaw.integral = 0;
                    yaw.offset = yaw.error;                
                }
                
                if(roll.error < 30 && roll.error > -30) {
                    altitude.output /= cos(TO_RAD(roll.error));
                }
                if(pitch.error < 30 && pitch.error > -30) {
                    altitude.output /= cos(TO_RAD(pitch.error));
                }

                //Motor Output
                
                altitude.output = LimitValue(altitude.output, 0.0f, 900.0f);

                speed.upRight   = altitude.output - pitch.output + roll.output - (yaw.output * MOTOR_SPIN);
                speed.downLeft  = altitude.output + pitch.output - roll.output - (yaw.output * MOTOR_SPIN);
                speed.upLeft    = altitude.output - pitch.output - roll.output + (yaw.output * MOTOR_SPIN);
                speed.downRight = altitude.output + pitch.output + roll.output + (yaw.output * MOTOR_SPIN);
                
                speed.upRight   += lidar.output;
                speed.downLeft  += lidar.output;
                speed.upLeft    += lidar.output;
                speed.downRight += lidar.output;

                LimitSpeed(&speed);

                //Output to ESC's
                if(!kill) {
                    WriteMotors(speed);
                } else { 
                    TurnMotorsOff();
                }
            }
        }
        TurnMotorsOff();
        
        LOOP_TIMER_ON = 0;        
    }
    
    return 0;
}