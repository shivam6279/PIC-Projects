#include "menu.h"
#include "PID.h"
#include "USART.h"
#include "SPI.h"
#include <stdbool.h>
#include "XBee.h"
#include "pic32.h"
#include "ToF.h"
#include "TF_Luna.h"
#include "GPS.h"
#include "AHRS.h"
#include "altitude.h"
#include "EEPROM.h"
#include "10DOF.h"
#include "MPU6050.h"
#include "QMC5883.h"
#include "LIS3MDL.h"
#include "BMP390.h"
#include "bitbang_I2C.h"
#include <xc.h>
#include <math.h>

void HSVtoRGB(int H, double S, double V, unsigned char *r, unsigned char *g, unsigned char *b) {
    double C = S * V;
    double X = C * (1.0 - fabs(fmod(H / 60.0, 2.0) - 1.0));
    double m = V - C;
    double Rs, Gs, Bs;

    if(H >= 0 && H < 60) {
        Rs = C;
        Gs = X;
        Bs = 0; 
    }
    else if(H >= 60 && H < 120) {   
        Rs = X;
        Gs = C;
        Bs = 0; 
    }
    else if(H >= 120 && H < 180) {
        Rs = 0;
        Gs = C;
        Bs = X; 
    }
    else if(H >= 180 && H < 240) {
        Rs = 0;
        Gs = X;
        Bs = C; 
    }
    else if(H >= 240 && H < 300) {
        Rs = X;
        Gs = 0;
        Bs = C; 
    }
    else {
        Rs = C;
        Gs = 0;
        Bs = X; 
    }
    
    float fr = ((float)(Rs + m) * 255.0f);
    float fg = ((float)(Gs + m) * 255.0f);
    float fb = ((float)(Bs + m) * 255.0f);
    
    *r = (unsigned char)fr;
    *g = (unsigned char)fg;
    *b = (unsigned char)fb;
}

void Menu(PID *x, PID *y, PID *z, PID *a){
    bool flag_menu = 1; 
    signed char cursor = 0;
    float delta;
    unsigned char r, g, b;
    int hue = 0;
    unsigned int arming_counter = 0;
    unsigned int cal_counter = 0;
    rx XBee;

    XBeeClearBuffer();
    
    while(arming_counter < 20) {
        
        XBee = ReadXBee();
        
        //-------------------------------------------------------------------LED stuff------------------------------------------------------------------------------
        
        HSVtoRGB(hue, 1.0, 1.0, &r, &g, &b);
        WriteRGBLed(r/50, g/50, b/50);
        hue += 1.2;
        if(hue >= 360) {
            hue = 0;
        }
        
        //----------------------------------------------------------------------------------------------------------------------------------------------------------
        if(XBee.y1 > 6 && flag_menu == 0) { 
            cursor--; 
            flag_menu = 1; 
        }
        else if(XBee.y1 < (-6) && flag_menu == 0) { 
            cursor++; 
            flag_menu = 1; 
        }
        else if(XBee.y1 > (-6) && XBee.y1 < 6) flag_menu = 0;
        
        if(cursor > 5) cursor = 5;
        else if(cursor < 0) cursor = 0;
        
        if(XBee.x1 > 4) {
            delta = ((float)XBee.x1 - 4.0)/(31.0 - 4.0) * 0.01 + 0.001;
        } else if(XBee.x1 < -4) {
            delta = ((float)XBee.x1 + 4.0)/(31.0 - 4.0) * 0.01 + 0.001;
        } else {
            delta = 0;
        }
        
        switch(cursor){
            case 0:
                x->kp += delta; 
                y->kp += delta;
                break;
            case 1:
                x->ki += delta; 
                y->ki += delta;
                break;
            case 2:
                x->kd += delta; 
                y->kd += delta;
                break;
            case 3:
                a->kp += delta*50; 
                break;
            case 4:
                a->ki += delta*10; 
                break;
            case 5:
                a->kd += delta*100;
                break;
        }
        
        if(XBee.ls && !XBee.rs && XBee.y2 < 2 && XBee.x2 > 13) {
            arming_counter++;
        } else { 
            arming_counter = 0;
        }
        
        if(!XBee.rs && XBee.y2 > 29 && XBee.x2 > 13) {
            cal_counter++;
            if(cal_counter >= 20) {
                Write_Onboard_LEDs(0, 255, 100);
                CalibrationMenu();
                cal_counter = 0;
            }
        } else {
            cal_counter = 0;
        }
        
        StartDelayCounter();
        
        while(!TxBufferEmpty());
        XBeePacketChar('A');
        XBeePacketInt(cursor);
        XBeePacketChar(',');
        XBeePacketFloat(x->kp, 3);
        XBeePacketChar(',');
        XBeePacketFloat(x->ki, 3);
        XBeePacketChar(',');
        XBeePacketFloat(x->kd, 3);
        XBeePacketChar(',');
        XBeePacketFloat(a->kp, 2);
        XBeePacketChar(',');
        XBeePacketFloat(a->ki, 2);
        XBeePacketChar(',');
        XBeePacketFloat(a->kd, 2);
        XBeePacketChar(',');
        XBeePacketInt(gps_connected);
        XBeePacketChar(',');
        XBeePacketInt(gps_signal);
        XBeePacketChar(',');        
        XBeePacketInt(arming_counter); 
        XBeePacketSend();
        
        while(ms_counter() < 25);  
        StopDelayCounter();
    }
//    z->p = x->p;
//    z->i = x->i;
    z->kd = x->kd;
}

void CalibrationMenu() {
    int cursor = 0, i;
    bool flag_menu = 1;
    bool p_ls;
    XYZ acc, gyro, compass;
    float q[4];
    float roll, pitch, heading;
    unsigned int sr_len = 0;
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
    
    XBeeClearBuffer();
    
    for(i = 0; i < 20; i++) {
        StartDelayCounter();
        XBeePacketChar('Z');
        XBeePacketStr("Calibration Menu");
        XBeePacketSend();
        while(ms_counter() < 50);
    }
    
    while(XBee.rs == 0) {
        if(XBee.x2 > 12 && flag_menu == 0) {
            if(cursor == 0) {
                SendCalibrationData();
                p_ls = XBee.ls;
                while(XBee.ls == p_ls && XBee.rs == 0) {
                    StartDelayCounter();
                    XBeePacketChar('Z');
                    XBeePacketStr("Compass calibration results\n");
                    XBeePacketStr("Offset: ");
                    XBeePacketFloat(compass_offset.x, 3); XBeePacketStr(", ");
                    XBeePacketFloat(compass_offset.y, 3); XBeePacketStr(", ");
                    XBeePacketFloat(compass_offset.z, 3); XBeePacketChar('\n');
                    XBeePacketStr("Gain: ");
                    XBeePacketFloat(compass_gain.x, 6); XBeePacketStr(", ");
                    XBeePacketFloat(compass_gain.y, 6); XBeePacketStr(", ");
                    XBeePacketFloat(compass_gain.z, 6);
                    XBeePacketSend();
                    while(ms_counter() < 50);
                }
                if(XBee.rs == 1)
                    break;
            } else if(cursor == 1) {
                for(i = 0; i <= 100; i++) {
                    StartDelayCounter();
                    GetRawAcc(&acc);
                    XBeePacketChar('Z');
                    XBeePacketStr("Place board on a perfectly level surface");
                    XBeePacketSend();
                    while(ms_counter() < 20);
                }
                
                p_ls = XBee.ls;
                while(p_ls == XBee.ls && XBee.rs == 0) {                    
                    StartDelayCounter();
                    GetRawAcc(&acc);
                    XBeePacketChar('Z');
                    XBeePacketStr("Place board on a perfectly level surface\n");
                    XBeePacketStr("Toggle left switch to end\n");
                    XBeePacketStr("X: ");
                    XBeePacketFloat(acc.x, 1); XBeePacketChar('\n');
                    XBeePacketStr("Y: ");
                    XBeePacketFloat(acc.y, 1); XBeePacketChar('\n');
                    XBeePacketStr("Z: ");
                    XBeePacketFloat(acc.z, 1);
                    XBeePacketSend();
                    while(ms_counter() < 50);
                }
                if(XBee.rs == 1)
                    break;

                acc_offset.x = acc.x;
                acc_offset.y = acc.y;
                acc_offset.z = acc.z;
                
                p_ls = XBee.ls;
                while(XBee.ls == p_ls && XBee.rs == 0) {                    
                    StartDelayCounter();
                    XBeePacketChar('Z');
                    XBeePacketStr("Acc offset results\n");
                    XBeePacketStr("X Offset: ");
                    XBeePacketFloat(acc_offset.x, 1); XBeePacketChar('\n');
                    XBeePacketStr("Y Offset: ");
                    XBeePacketFloat(acc_offset.y, 1); XBeePacketChar('\n');
                    XBeePacketStr("Z Offset: ");
                    XBeePacketFloat(acc_offset.z, 1);
                    XBeePacketSend();
                    while(ms_counter() < 50);
                }
                if(XBee.rs == 1)
                    break;
                
            } else if(cursor == 2) {
                for(i = 0; i <= 100; i++) {
                    StartDelayCounter();
                    GetAcc(&acc);
                    GetCompass(&compass);

                    MadgwickQuaternionUpdate(q, acc, (XYZ){0.0, 0.0, 0.0}, compass, 0.050);
                    XBeePacketStr("ZPlace board on a perfectly level surface\nAnd point it north");
                    XBeePacketSend();

                    while(ms_counter() < 20);
                }
                
                roll_offset = 0;
                pitch_offset = 0;
                heading_offset = 0;
                
                p_ls = XBee.ls;
                while(p_ls == XBee.ls && XBee.rs == 0) {
                    StartDelayCounter();
                    GetAcc(&acc);
                    GetGyro(&gyro);
                    GetCompass(&compass);
                    MadgwickQuaternionUpdate(q, acc, gyro, compass, 0.050);
                    QuaternionToEuler(q, &roll, &pitch, &heading);
                    XBeePacketChar('Z');
                    XBeePacketStr("Place board on a perfectly level surface\nAnd point it north\n");
                    XBeePacketStr("Toggle left switch to end\n");
                    XBeePacketStr("Pitch: ");
                    XBeePacketFloat(pitch, 3); XBeePacketChar('\n');
                    XBeePacketStr("Roll: ");
                    XBeePacketFloat(roll, 3); XBeePacketChar('\n');
                    XBeePacketStr("Heading: ");
                    XBeePacketFloat(heading, 3);
                    XBeePacketSend();
                    while(ms_counter() < 50);
                }
                if(XBee.rs == 1)
                    break;
                
                roll_offset = roll;
                pitch_offset = pitch;
                heading_offset = heading;

                eeprom_writeOffsets();
                
                p_ls = XBee.ls;
                while(XBee.ls == p_ls && XBee.rs == 0) {
                    StartDelayCounter();
                    
                    XBeePacketChar('Z');
                    XBeePacketStr("Angle offset results\n");
                    XBeePacketStr("Pitch Offset: ");
                    XBeePacketFloat(pitch_offset, 3); XBeePacketChar('\n');
                    XBeePacketStr("Roll Offset: ");
                    XBeePacketFloat(roll_offset, 3); XBeePacketChar('\n');
                    XBeePacketStr("Heading Offset: ");
                    XBeePacketFloat(heading_offset, 3);
                    XBeePacketSend();
                    while(ms_counter() < 25);
                }
                if(XBee.rs == 1)
                    break;
            } else if(cursor == 3) {
                sr_len = 0;
            } else if(cursor == 4) {
                p_ls = XBee.ls;
                while(XBee.ls == p_ls && XBee.rs == 0) {
                    sr_len = 0;

                    bool i2c_flag = false;
                    
                    XBeePacketChar('Z');
                    
                    if(I2C_CheckAddress(EEPROM_ADDRESS)) {
                        i2c_flag = true;
                        XBeePacketStr("EEPROM connected\n");
                        sr_len += 17;
                    }
                    if(I2C_CheckAddress(MPU6050_ADDR)) {
                        i2c_flag = true;
                        XBeePacketStr("MPU6050 connected\n");
                        sr_len += 18;
                    }
                    if(I2C_CheckAddress(QMC5883_ADDR)) {
                        i2c_flag = true;
                        XBeePacketStr("QMC5883 connected\n");
                        sr_len += 18;
                    }
                    if(I2C_CheckAddress(LIS3MDL_ADDR)) {
                        i2c_flag = true;
                        XBeePacketStr("LIS3MDL connected\n");
                        sr_len += 18;
                    }
                    if(I2C_CheckAddress(BMP390_ADDR)) {
                        i2c_flag = true;
                        XBeePacketStr("BMP390 connected\n");
                        sr_len += 17;
                    }
                    if(I2C_CheckAddress(VL6180X_ADDRESS)) {
                        i2c_flag = true;
                        XBeePacketStr("VL6180X connected\n");
                        sr_len += 18;
                    } 
                    if(I2C_CheckAddress(TF_LUNA_ADDR)) {
                        i2c_flag = true;
                         XBeePacketStr("TF Luna connected\n");
                        sr_len += 18;
                    }

                    StartDelayCounter();                
                    if(i2c_flag == false) {
                        XBeePacketStr("No devices connected!");
                        XBeePacketSend();
                    }
                    XBeePacketSend();
                    
                    while(ms_counter() < 150);
                }
            }
        }
        if(XBee.y1 > 12 && flag_menu == 0) { 
            cursor--; 
            flag_menu = 1; 
        }
        else if(XBee.y1 < (-12) && flag_menu == 0) { 
            cursor++; 
            flag_menu = 1; 
        }
        else if(XBee.y1 > (-12) && XBee.y1 < 12 && XBee.x1 > (-12) && XBee.x1 < 12) flag_menu = 0;
        if(cursor < 0)
            cursor = 4;
        else if(cursor > 4) 
            cursor = 0;
        
        StartDelayCounter();
        
        XBeePacketChar('Z');
        XBeePacketStr("Calibration Menu\n");
        XBeePacketStr("Select an Option:\n");
        if(cursor == 0)
            XBeePacketStr("->");
        else
            XBeePacketStr("  ");
        XBeePacketStr("Calibrate Compass\n");
        if(cursor == 1)
            XBeePacketStr("->");
        else
            XBeePacketStr("  ");
        XBeePacketStr("Calibrate Accelerometer\n");
        if(cursor == 2)
            XBeePacketStr("->");
        else
            XBeePacketStr("  ");
        XBeePacketStr("Calibrate pitch and roll offsets\n");
        if(cursor == 3)
            XBeePacketStr("->");
        else
            XBeePacketStr("  ");
        XBeePacketStr("Change constants\n");
        if(cursor == 4)
            XBeePacketStr("->");
        else
            XBeePacketStr("  ");
        XBeePacketStr("View I2C devices");
        XBeePacketSend();
        
        while(ms_counter() < 50);
    }
}

void ArmingSequence(float q[], float *gravity_mag, float *to_roll, float *to_pitch, float *to_heading, float *to_altitude, float *to_latitude, float *to_longitude) {         
    int i;
    const int ITERATIONS = 1500;
    XYZ acc = {0.0, 0.0, 0.0}, gyro = {0.0, 0.0, 0.0}, compass = {0.0, 0.0, 0.0};
    XYZ gravity;
    
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
    
    WriteRGBLed(255, 200, 0); //Yellow
    
    delay_ms(100);
    
    VectorReset(&gravity);
    VectorReset(&gyro_offset);
                
    for(i = 0; i < ITERATIONS; i++) {
        StartDelayCounter();
        
        GetAcc(&acc);
        GetRawGyro(&gyro);
        GetCompass(&compass);
        
        gravity = VectorAdd(gravity, acc);
        gyro_offset = VectorAdd(gyro_offset, gyro);
        
        MadgwickQuaternionUpdate(q, acc, (XYZ){0.0, 0.0, 0.0}, compass, 0.1);
        
        if(i % 5 == 0) {
            XBeePacketChar('B');
            XBeePacketInt(i);
            XBeePacketSend();
        }
        
        while(ms_counter() < 2);
    }
    StopDelayCounter();
    
    gravity = VectorScale(gravity, 1 / (float)ITERATIONS);
    gyro_offset = VectorScale(gyro_offset, 1 / (float)ITERATIONS);
    
    *gravity_mag = sqrt(gravity.x * gravity.x + gravity.y * gravity.y + gravity.z * gravity.z);

    //Read initial heading
    QuaternionToEuler(q, to_roll, to_pitch, to_heading);

    //Read take-off altitude
    altitude_KF_reset();
    *to_altitude = GetTakeoffAltitude();
    altitude_KF_setAltitude(*to_altitude);
    altitude_KF_update(*to_altitude);
    
    if(gps_signal) { 
        *to_latitude = latitude; 
        *to_longitude = longitude; 
    } else { 
        *to_latitude = 0.0; 
        *to_longitude = 0.0; 
    }
}