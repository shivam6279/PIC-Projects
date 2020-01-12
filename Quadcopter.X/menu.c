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
    unsigned char r, g, b;
    int hue = 0;
    unsigned int arming_counter = 0;
    rx XBee;

    XBeeClearBuffer();
    
    while(arming_counter < 20) {
        
        XBee = ReadXBee();
        
        //-------------------------------------------------------------------LED stuff------------------------------------------------------------------------------
        
        HSVtoRGB(hue, 1.0, 1.0, &r, &g, &b);
        WriteRGBLed(r, g, b);
        hue += 2;
        if(hue >= 360)
            hue = 0;
        
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
    
    for(i = 0; i < 20; i++) {
        StartDelayCounter();
        XBeeWriteChar('Z');
        XBeeWriteStr("Calibration Menu\r");
        while(ms_counter() < 50);
    }
    
    while(XBee.rs == 0) {
        if(XBee.x2 > 12 && flag_menu == 0) {
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

                eeprom_writeOffsets();
                
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
        XBeeWriteStr("Calibrate pitch and roll offsets\n");
        if(cursor == 3)
            XBeeWriteStr("->");
        else
            XBeeWriteStr("  ");
        XBeeWriteStr("Change constants\r");
        
        while(ms_counter() < 50);
    }
}

void ArmingSequence(float q[], float *gravity_mag, float *to_roll, float *to_pitch, float *to_heading, float *to_altitude, float *to_latitude, float *to_longitude) {         
    int i;
    const int ITERATIONS = 1000;
    XYZ acc, gyro, compass;
    XYZ gravity;
    
    WriteRGBLed(255, 200, 0); //Yellow
    
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
    QuaternionToEuler(q, to_roll, to_pitch, to_heading);

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