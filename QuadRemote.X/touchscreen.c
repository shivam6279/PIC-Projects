#include "touchscreen.h"
#include "settings.h"
#include <xc.h>

void GetTouchscreen() {
    int i;
    //Set Y resistors as outputs
    TRISEbits.TRISE5 = 0;   //y1
    TRISEbits.TRISE6 = 0;   //y2
    //Set x resistors as inputs
    TRISEbits.TRISE4 = 1;   //x1
    TRISEbits.TRISE7 = 1;   //x2
    //
    LATEbits.LATE5 = 0;     //y1 = -
    LATEbits.LATE6 = 1;     //y2 = +
    
    ANSELEbits.ANSE4 = 1;
    ANSELEbits.ANSE5 = 0;
    
    ts_x = 0;
    for(i = 0; i < 10; i++){
        ADCCON3bits.GSWTRG = 1;
        while(ADCDSTAT1bits.ARDY18 == 0);
        ts_x += ADCDATA18;
    }
    ts_x /= 10;
    
    //Set X resistors as outputs
    TRISEbits.TRISE5 = 1;   //y1
    TRISEbits.TRISE6 = 1;   //y2
    //Set Y resistors as inputs
    TRISEbits.TRISE4 = 0;   //x1
    TRISEbits.TRISE7 = 0;   //x2
    //
    LATEbits.LATE4 = 1;     //x1 = -
    LATEbits.LATE7 = 0;     //x2 = +
    
    ANSELEbits.ANSE4 = 0;
    ANSELEbits.ANSE5 = 1;
    
    ts_y = 0;
    for(i = 0; i < 10; i++){
        ADCCON3bits.GSWTRG = 1;
        while(ADCDSTAT1bits.ARDY17 == 0);
        ts_y += ADCDATA17;
    }
    ts_y /= 10;
    
    ts_x = (320.0 * (float)(ts_x - ts_x_min) / (float)(ts_x_max - ts_x_min));
    ts_y = (240.0 * (float)(ts_y - ts_y_min) / (float)(ts_y_max - ts_y_min));
    if(ts_x > 319) ts_x = 319;
    else if(ts_x < 0) ts_x = 0;   
    if(ts_y > 239) ts_y = 239;
    else if(ts_y < 0) ts_y = 0;
}