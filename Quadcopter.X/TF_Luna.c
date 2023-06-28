#include "TF_luna.h"
#include "bitbang_I2C.h"

void TF_luna_getData(int *dist, int *flux) {
    static unsigned char arr[6] = {0, 0, 0, 0};
    I2C_ReadRegisters(TF_LUNA_ADDR, TFL_DIST_LO , arr, 4);
    
    *dist = arr[1] << 8 | arr[0];
    *flux = arr[3] << 8 | arr[2];
    
    if(*flux < 100 || *flux == 0xFFFF) {
        *dist = -1;
    }
}
