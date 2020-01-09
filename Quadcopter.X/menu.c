#include "menu.h"
#include "PID.h"
#include "USART.h"
#include <stdbool.h>
#include "XBee.h"
#include "pic32.h"
#include "GPS.h"
#include "AHRS.h"
#include "altitude.h"
#include <xc.h>
#include <math.h>

void Menu(PID *x, PID *y, PID *z, PID *a){
    bool flag_menu = 1; 
    signed char cursor = 0; 
    unsigned int r, g, b;
    unsigned int led_counter = 0, arming_counter = 0;
    rx XBee;

    XBeeClearBuffer();
    
    while(arming_counter < 20) {
        
        XBee = ReadXBee();
        
        //-------------------------------------------------------------------LED stuff------------------------------------------------------------------------------
        
        if(led_counter < 4096){  r = 4095; g = led_counter; b = 0; }
        else if(led_counter < 8192) { 
            r = 4095 - (led_counter - 4096); 
            g = 4095; b = 0; 
        }
        else if(led_counter < 12288) { 
            r = 0; g = 4095; 
            b = led_counter - 8192; 
        }
        else if(led_counter < 16384) { 
            r = 0; g = 4095 - (led_counter - 12288); 
            b = 4095; 
        }
        else if(led_counter < 20480) { 
            r = led_counter - 16384; 
            g = 0; b = 4095; 
        } else { 
            r = 4095; 
            g = 0; 
            b = 4095 - (led_counter - 20480); 
        }
        led_counter += 150;
        if(led_counter >= 24576) led_counter = 0;
        WriteRGBLed(r, g, b);
        
        //----------------------------------------------------------------------------------------------------------------------------------------------------------
        if(XBee.y1 > 12 && flag_menu == 0) { 
            cursor--; 
            flag_menu = 1; 
        }
        else if(XBee.y1 < (-12) && flag_menu == 0) { 
            cursor++; 
            flag_menu = 1; 
        }
        else if(XBee.y1 > (-12) && XBee.y1 < 12) flag_menu = 0;
        
        if(cursor > 5) cursor = 5;
        else if(cursor < 0) cursor = 0;
        
        switch(cursor){
            case 0:
                if(XBee.x1 > 12) {
                    x->p += 0.01; 
                    y->p += 0.01;
                }
                else if(XBee.x1 < (-12)) {
                    x->p -= 0.01; 
                    y->p -= 0.01;
                }
                break;
            case 1:
                if(XBee.x1 > 12) { 
                    x->i += 0.01; 
                    y->i += 0.01;
                }
                else if(XBee.x1 < (-12)) { 
                    x->i -= 0.01; 
                    y->i -= 0.01;
                }
                break;
            case 2:
                if(XBee.x1 > 12) { 
                    x->d += 0.001; 
                    y->d += 0.001;
                }
                else if(XBee.x1 < (-12)) { 
                    x->d -= 0.001; 
                    y->d -= 0.001;
                }
                break;
            case 3:
                if(XBee.x1 > 12) a->p += 0.2; 
                else if(XBee.x1 < (-12)) a->p -= 0.2;
                break;
            case 4:
                if(XBee.x1 > 12) a->i += 0.01; 
                else if(XBee.x1 < (-12)) a->i -= 0.01; 
                break;
            case 5:
                if(XBee.x1 > 12) a->d += 1; 
                else if(XBee.x1 < (-12)) a->d -= 1;
                break;
        }
        
        if(XBee.ls && !XBee.rs && XBee.y2 < 2 && XBee.x2 > 13) 
            arming_counter++;
        else 
            arming_counter = 0;
        
        StartDelayCounter();

        XBeeWriteChar('A');
        XBeeWriteInt(cursor); XBeeWriteChar(',');
        XBeeWriteFloat(x->p, 3); XBeeWriteChar(',');
        XBeeWriteFloat(x->i, 3); XBeeWriteChar(',');
        XBeeWriteFloat(x->d, 3); XBeeWriteChar(',');
        XBeeWriteFloat(a->p, 2); XBeeWriteChar(',');
        XBeeWriteFloat(a->i, 2); XBeeWriteChar(',');
        XBeeWriteFloat(a->d, 2); XBeeWriteChar(',');
        XBeeWriteInt(GPS_connected); XBeeWriteChar(',');
        XBeeWriteInt(GPS_signal); XBeeWriteChar(',');        
        XBeeWriteInt(arming_counter); 
        XBeeWriteChar('\r');
        
        while(ms_counter() < 25);  
        StopDelayCounter();
    }
//    z->p = x->p;
//    z->i = x->i;
    z->d = x->d;
}

void CalibrationMenu() {
    int cursor = 0, i;
    bool flag_menu = 1;
    bool p_ls;
    XYZ acc, gyro, compass;
    float q[4];
    float roll, pitch, heading;
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
    
    for(i = 0; i < 10; i++) {
        StartDelayCounter();
        XBeeWriteChar('Z');
        XBeeWriteStr("Calibration Menu\r");
        while(ms_counter() < 50);
    }
    
    while(XBee.rs == 0) {
        if(XBee.x1 > 12 & flag_menu == 0) {
            if(cursor == 0) {
                SendCalibrationData();
                p_ls = XBee.ls;
                while(XBee.ls == p_ls && XBee.rs == 0) {
                    StartDelayCounter();
                    XBeeWriteChar('Z');
                    XBeeWriteStr("Compass calibration results\n");
                    XBeeWriteStr("Offset: ");
                    XBeeWriteFloat(compass_offset.x, 3); XBeeWriteStr(", ");
                    XBeeWriteFloat(compass_offset.y, 3); XBeeWriteStr(", ");
                    XBeeWriteFloat(compass_offset.z, 3); XBeeWriteChar('\n');
                    XBeeWriteStr("Gain: ");
                    XBeeWriteFloat(compass_gain.x, 6); XBeeWriteStr(", ");
                    XBeeWriteFloat(compass_gain.y, 6); XBeeWriteStr(", ");
                    XBeeWriteFloat(compass_gain.z, 6); XBeeWriteChar('\r');
                    while(ms_counter() < 50);
                }
                if(XBee.rs == 1)
                    break;
            } else if(cursor == 1) {
                for(i = 0; i <= 100; i++) {
                    StartDelayCounter();
                    acc = GetRawAcc();
                    XBeeWriteChar('Z');
                    XBeeWriteStr("Place board on a perfectly level surface\r");
                    while(ms_counter() < 20);
                }
                
                p_ls = XBee.ls;
                while(p_ls == XBee.ls && XBee.rs == 0) {
                    StartDelayCounter();
                    acc = GetRawAcc();
                    XBeeWriteChar('Z');
                    XBeeWriteStr("Place board on a perfectly level surface\n");
                    XBeeWriteStr("Toggle left switch to end");
                    XBeeWriteStr("X: ");
                    XBeeWriteFloat(acc.x, 1); XBeeWriteChar('\n');
                    XBeeWriteStr("Y: ");
                    XBeeWriteFloat(acc.y, 1); XBeeWriteChar('\n');
                    XBeeWriteStr("Z: ");
                    XBeeWriteFloat(acc.z, 1); XBeeWriteChar('\r');
                    while(ms_counter() < 20);
                }
                if(XBee.rs == 1)
                    break;

                acc_offset.x = acc.x;
                acc_offset.y = acc.y;
                acc_offset.z = acc.z;
                
                p_ls = XBee.ls;
                while(XBee.ls == p_ls && XBee.rs == 0) {
                    StartDelayCounter();
                    XBeeWriteChar('Z');
                    XBeeWriteStr("Acc offset results\n");
                    XBeeWriteStr("Pitch Offset: ");
                    XBeeWriteFloat(acc_offset.x, 1); XBeeWriteChar('\n');
                    XBeeWriteStr("Roll Offset: ");
                    XBeeWriteFloat(acc_offset.y, 1); XBeeWriteChar('\n');
                    XBeeWriteStr("Heading Offset: ");
                    XBeeWriteFloat(acc_offset.z, 1); XBeeWriteChar('\r');
                    while(ms_counter() < 50);
                }
                if(XBee.rs == 1)
                    break;
                
            } else if(cursor == 2) {
                for(i = 0; i <= 100; i++) {
                    StartDelayCounter();
                    acc = GetAcc();
                    compass = GetCompass();

                    MadgwickQuaternionUpdate(q, acc, (XYZ){0.0, 0.0, 0.0}, compass, 0.050);
                    XBeeWriteChar('Z');
                    XBeeWriteStr("Place board on a perfectly level surface and point it north\r");

                    while(ms_counter() < 20);
                }
                
                roll_offset = 0;
                pitch_offset = 0;
                heading_offset = 0;
                
                p_ls = XBee.ls;
                while(p_ls == XBee.ls && XBee.rs == 0) {
                    StartDelayCounter();
                    acc = GetAcc();
                    compass = GetCompass();

                    MadgwickQuaternionUpdate(q, acc, (XYZ){0.0, 0.0, 0.0}, compass, 0.050);
                    QuaternionToEuler(q, &roll, &pitch, &heading);
                    XBeeWriteChar('Z');
                    XBeeWriteStr("Place board on a perfectly level surface and point it north\n");
                    XBeeWriteStr("Toggle left switch to end");
                    XBeeWriteStr("Pitch: ");
                    XBeeWriteFloat(pitch, 3); XBeeWriteChar('\n');
                    XBeeWriteStr("Roll: ");
                    XBeeWriteFloat(pitch, 3); XBeeWriteChar('\n');
                    XBeeWriteStr("Heading: ");
                    XBeeWriteFloat(heading, 3); XBeeWriteChar('\r');
                    while(ms_counter() < 20);
                }
                if(XBee.rs == 1)
                    break;
                
                roll_offset = roll;
                pitch_offset = pitch;
                heading_offset = heading;
                
                p_ls = XBee.ls;
                while(XBee.ls == p_ls && XBee.rs == 0) {
                    StartDelayCounter();
                    XBeeWriteChar('Z');
                    XBeeWriteStr("Angle offset results\n");
                    XBeeWriteStr("Pitch Offset: ");
                    XBeeWriteFloat(pitch, 3); XBeeWriteChar('\n');
                    XBeeWriteStr("Roll Offset: ");
                    XBeeWriteFloat(roll, 3); XBeeWriteChar('\n');
                    XBeeWriteStr("Heading Offset: ");
                    XBeeWriteFloat(heading, 3); XBeeWriteChar('\r');
                    while(ms_counter() < 50);
                }
                if(XBee.rs == 1)
                    break;
            } else if(cursor == 3) {
                
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
            cursor = 3;
        else if(cursor > 3) 
            cursor = 0;
        
        StartDelayCounter();
        
        XBeeWriteChar('Z');
        XBeeWriteStr("Calibration Menu\n");
        XBeeWriteStr("Select an Option:\n");
        if(cursor == 0)
            XBeeWriteStr("->");
        else
            XBeeWriteStr("  ");
        XBeeWriteStr("Calibrate Compass\n");
        if(cursor == 1)
            XBeeWriteStr("->");
        else
            XBeeWriteStr("  ");
        XBeeWriteStr("Calibrate Accelerometer\n");
        if(cursor == 2)
            XBeeWriteStr("->");
        else
            XBeeWriteStr("  ");
        XBeeWriteStr("Calibrate pitch and roll offsets (after 1 and 2)\n");
        if(cursor == 3)
            XBeeWriteStr("->");
        else
            XBeeWriteStr("  ");
        XBeeWriteStr("Change constants\r");
        
        while(ms_counter() < 25);
    }
}

void ArmingSequence(float q[], float *gravity_mag, float *to_heading, float *to_altitude, float *to_latitude, float *to_longitude) {         
    int i;
    const int ITERATIONS = 1000;
    XYZ acc, gyro, compass;
    XYZ gravity;
    
    WriteRGBLed(4095, 2500, 0); //Yellow
    
    delay_ms(100);
    
    for(i = 0, VectorReset(&gravity), VectorReset(&gyro_offset); i < ITERATIONS; i++) {
        StartDelayCounter();

        acc = GetAcc();
        gyro = GetRawGyro();
        compass = GetCompass();
        
        gravity = VectorAdd(gravity, acc);
        gyro_offset = VectorAdd(gyro_offset, gyro);
        
        MadgwickQuaternionUpdate(q, acc, (XYZ){0.0, 0.0, 0.0}, compass, 0.050);
        
        XBeeWriteChar('B');
        XBeeWriteInt(i);
        XBeeWriteChar('\r');
        
        while(ms_counter() < 3);
    }
    StopDelayCounter();
    
    gravity = VectorScale(gravity, 1 / (float)ITERATIONS);
    gyro_offset = VectorScale(gyro_offset, 1 / (float)ITERATIONS);
    
    *gravity_mag = sqrt(gravity.x * gravity.x + gravity.y * gravity.y + gravity.z * gravity.z);

    //Read initial heading
    *to_heading = LimitAngle(-atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * RAD_TO_DEGREES - heading_offset);

    //Read take-off altitude
    altitude_KF_reset();
    *to_altitude = GetTakeoffAltitude();
    
    if(GPS_signal) { 
        *to_latitude = latitude; 
        *to_longitude = longitude; 
    } else { 
        *to_latitude = 0.0; 
        *to_longitude = 0.0; 
    }
}