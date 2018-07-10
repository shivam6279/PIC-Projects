#include "inputdata.h"
#include "touchscreen.h"

void ShowInputData() {
    int i;
    static int pre_x1 = 10;
    static int pre_y1 = 10;
    int tx1, ty1;
    static int pre_x2 = 10;
    static int pre_y2 = 10;
    int tx2, ty2;
    static int pre_dial1 = 3;
    static int pre_dial2 = 3;
    static int pre_ts_x = 0;
    static int pre_ts_y = 0;
    
    ty2 = analog2_y;
    if(ty2 != pre_y2) {
        if(ty2 > pre_y2) {
            FillRect(270, (float)(31 - ty2) * 240.0/31.0, 50, 1 + (float)(ty2 - pre_y2) * 240.0/31.0, 0xFFA0);
        } else {
            FillRect(270, (float)(31 - pre_y2) * 240.0/31.0, 50, 1 + (float)(pre_y2 - ty2) * 240.0/31.0,0xFFFF);
        }
        pre_y2 = ty2;
    }

    tx1 = analog1_x;
    ty1 = analog1_y;
    if(pre_x1 != tx1 || pre_y1 != ty1) {
        for(i = 0; i <= 5; i++) DrawCircle(68 + ((float)pre_x1 * 2.4), 178 - ((float)pre_y1 * 2.4), i, 0xFFFF);
        FillRect(27, 178, 83, 1, 0xF000);
        FillRect(68.5, 137, 1, 83, 0xF000);
        for(i = 0; i <= 5; i++) DrawCircle(68 + ((float)tx1 * 2.4), 178 - ((float)ty1 * 2.4), i, 0x0000);
        pre_x1 = tx1;
        pre_y1 = ty1;
    }

    GetTouchscreen();
    if(ts_y > 145 && ts_y < (145 + 20)) {
        if(ts_x > 116 && ts_x < (116 + 18)) dial1 = 0;
        else if(ts_x > (116 + 21) && ts_x < (116 + 18 + 21)) dial1 = 1;
        else if(ts_x > (116 + 21 * 2) && ts_x < (116 + 18 + 21 * 2)) dial1 = 2;

        else if(ts_x > 186 && ts_x < (186 + 18)) dial2 = 0;
        else if(ts_x > (186 + 21) && ts_x < (186 + 18 + 21)) dial2 = 1;
        else if(ts_x > (186 + 21 * 2) && ts_x < (186 + 18 + 21 * 2)) dial2 = 2;
    }

    if(pre_dial1 != dial1) {
        if(pre_dial1 >=0 && pre_dial1 < 3) FillRect(116 + (pre_dial1 * 21), 145, 18, 20, 0xFFFF);
        if(dial1 == 0) FillRect(116 + (dial1 * 21), 145, 18, 20, 0xF800);
        else if(dial1 == 1) FillRect(116 + (dial1 * 21), 145, 18, 20, 0x07E0);
        else if(dial1 == 2) FillRect(116 + (dial1 * 21), 145, 18, 20, 0x001F);
    }
    if(pre_dial2 != dial2) {
        if(pre_dial2 >=0 && pre_dial2 < 3) FillRect(186 + (pre_dial2 * 21), 145, 18, 20, 0xFFFF);
        if(dial2 == 0) FillRect(186 + (dial2 * 21), 145, 18, 20, 0xF800);
        else if(dial2 == 1) FillRect(186 + (dial2 * 21), 145, 18, 20, 0x07E0);
        else if(dial2 == 2) FillRect(186 + (dial2 * 21), 145, 18, 20, 0x001F);
    }
    pre_dial1 = dial1;
    pre_dial2 = dial2;

    tx2 = analog2_x;
    if(tx2 != pre_x2) WriteInt(analog2_x, 2, 24 * 6, 23 * 8, 0x0000);
    pre_x2 = tx2;

    WriteInt(!switch1, 1, 24 * 6, 24 * 8, 0x0000);
    WriteInt(!switch2, 1, 24 * 6, 25 * 8, 0x0000);     
}

void DrawDisplayBounds() {
    WriteStr("Dial2", 200, 135, 0x0000);//Dial2    
    FillRect(267, 0, 3, 320, 0xF000);//Throttle divide
    FillRect(25, 135, 87, 87, 0xF000);//Analog indicator box
    FillRect(27, 137, 83, 83, 0xFFFF);//?
    FillRect(114, 143, 64, 24, 0x0000);//Dial 1 box
    FillRect(116, 145, 60, 20, 0xFFFF);//?
    FillRect(134, 145, 3, 20, 0x0000);//?
    FillRect(155, 145, 3, 20, 0x0000);//?
    WriteStr("Dial1", 130, 135, 0x0000);//Dial1
    FillRect(184, 143, 64, 24, 0x0000);//Dial2 box
    FillRect(186, 145, 60, 20, 0xFFFF);//?
    FillRect(204, 145, 3, 20, 0x0000);//?
    FillRect(225, 145, 3, 20, 0x0000);//?
}