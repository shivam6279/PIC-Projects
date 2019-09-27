#include "debug.h"

void debug_int(long int t, struct led *buffer, enum color c){
    int i;
    for (i = 0; i < 32; i++) {
        if (t & 1) {
            if(c == RED) buffer[i].red = 255;
            if(c == GREEN) buffer[i].green = 255;
            if(c == BLUE) buffer[i].blue = 255;
        } else {
            if(c == RED) buffer[i].red = 0;
            if(c == GREEN) buffer[i].green = 0;
            if(c == BLUE) buffer[i].blue = 0;
        }
        t = t >> 1;
    }
}