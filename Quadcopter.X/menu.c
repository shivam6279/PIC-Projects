#include "menu.h"
#include "PID.h"
#include "USART.h"
#include <stdbool.h>
#include "XBee.h"
#include "pic32.h"
#include "GPS.h"
#include <xc.h>

void Menu(PID *x, PID *y, PID *z, PID *a){
    bool flag_menu = 1; 
    signed char cursor = 0; 
    unsigned int r, g, b;
    unsigned int led_counter = 0, arming_counter = 0;
    rx XBee;
    
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
                    z->p += 0.01;
                }
                else if(XBee.x1 < (-12)) {
                    x->p -= 0.01; 
                    y->p -= 0.01; 
                    z->p -= 0.01;
                }
                break;
            case 1:
                if(XBee.x1 > 12) { 
                    x->i += 0.01; 
                    y->i += 0.01; 
                    z->i += 0.01;
                }
                else if(XBee.x1 < (-12)) { 
                    x->i -= 0.01; 
                    y->i -= 0.01; 
                    z->i -= 0.01;
                }
                break;
            case 2:
                if(XBee.x1 > 12) { 
                    x->d += 0.01; 
                    y->d += 0.01; 
                    z->d += 0.01;
                }
                else if(XBee.x1 < (-12)) { 
                    x->d -= 0.01; 
                    y->d -= 0.01; 
                    z->d -= 0.01;
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
        
        delay_counter = 0; 
        T2CONbits.TON = 1;
        GetGyro();
        USART1_send('A');
        USART1_send((cursor % 10) + 48);        USART1_send(',');
        USART1_write_float(x->p, 2, 2);         USART1_send(',');
        USART1_write_float(x->i, 2, 2);         USART1_send(',');
        USART1_write_float(x->d, 2, 2);         USART1_send(',');
        USART1_write_float(a->p, 3, 1);         USART1_send(',');
        USART1_write_float(a->i, 2, 2);         USART1_send(',');
        USART1_write_float(a->d, 3, 1);         USART1_send(',');
        USART1_send((GPS_signal % 10) + 48);    USART1_send(',');
        USART1_send((GPS_connected % 10) + 48); USART1_send(',');
        USART1_send(((arming_counter / 10) % 10) + 48); 
        USART1_send((arming_counter % 10) + 48);
        USART1_send('\r');
        while(delay_counter < 25); 
        T2CONbits.TON = 0;
    }
}