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

void ArmingSequence(float q[], float *gravity_mag, float *to_heading, float *to_altitude, float *to_latitude, float *to_longitude) {         
    int i;
    XYZ acc, gyro, compass;
    XYZ gravity;
    
    WriteRGBLed(4095, 2500, 0); //Yellow
    
    delay_ms(100);
    
    for(i = 0, VectorReset(&gravity), VectorReset(&gyro_offset); i < 1000; i++) {
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
    
    gravity = VectorScale(gravity, 1 / 1000.0);
    gyro_offset = VectorScale(gyro_offset, 1 / 1000.0);
    
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