#include "pragma.h"

#include <xc.h>
#include <proc/p32mz2064das176.h>
#include <math.h>
#include <stdbool.h>
#include <sys/attribs.h>
#include <string.h>

#include "settings.h"
#include "pic32.h"
#include "plib_ddr.h"
#include "plib_glcd.h"

#define  GFX_GLCD_LAYER0_BASEADDR 0xA8000000

void GLCD_init() {    
    GLCDMODEbits.LCDEN = 0;           
    CFGCON2bits.GLCDPINEN = 0; 
    GLCDBGCOLOR |= (0x10UL << 31) | (0xFFUL << 23) | (0xFFUL << 15) | (0xF0UL);
    GLCDINTbits.VSYNCINT = 0;
    GLCDINTbits.HSYNCINT = 0;
    CFGCON2bits.GLCDMODE = 0;
    GLCDMODEbits.RGBSEQ = 0;
    GLCDFPORCHbits.FPORCHX = 0;
    GLCDFPORCHbits.FPORCHY = 0;
    GLCDBLANKINGbits.BLANKINGX = 0;
    GLCDBLANKINGbits.BLANKINGY = 0;
    GLCDBPORCHbits.BPORCHX = 0;
    GLCDBPORCHbits.BPORCHY = 0;
    GLCDCLKCON |= 6UL;
    GLCDRESbits.RESX = 800;
    GLCDRESbits.RESY = 480;
    GLCDMODEbits.LCDEN = 1;                
    CFGCON2bits.GLCDPINEN = 1;  
    
    /*CFGCON2bits.GLCDMODE = 0;
    CFGCON2bits.GLCDPINEN = 1;
    
    GLCDMODE = 0;
    GLCDCLKCON |= 4UL;
    
    GLCDBGCOLOR |= (0x10UL << 31) | (0xFFUL << 23) | (0xFFUL << 15) | (0xF0UL);

//    
    GLCDRESbits.RESX = 800;
    GLCDRESbits.RESY = 480;
    
    GLCDFPORCHbits.FPORCHX = 0;
    GLCDFPORCHbits.FPORCHY = 0;
    
    GLCDBPORCHbits.BPORCHX = 0;
    GLCDBPORCHbits.BPORCHY = 0;
//    
    GLCDBLANKINGbits.BLANKINGX = 0;
    GLCDBLANKINGbits.BLANKINGY = 0;
    
    GLCDMODEbits.LCDEN = 1;*/
}

int main() {
    unsigned long int i;
    PICInit();
    TRISBbits.TRISB9 = 0;
    TRISDbits.TRISD7 = 0;
    LATDbits.LATD7 = 0;
    
    /*CFGMPLLbits.MPLLVREGDIS = 0;
    while(CFGMPLLbits.MPLLVREGRDY == 0U);
    CFGMPLL = 0x0B001901;    
    while(CFGMPLLbits.MPLLRDY == 0U);    
    CFGCONbits.PMDLOCK = 0;
    PMD7 = 0x500000U;
    CFGCONbits.PMDLOCK = 1;*/

    /* Lock system since done with clock configuration */
//    SYSKEY = 0x33333333U;
    
//    DDR_Initialize();
    timer2_init(1000);
    
//    timer3_init(50000000);
//    GLCD_init();
    
    TRISBbits.TRISB9 = 0;
    while(1) {
        LATBbits.LATB9 = 1;
        for(i = 0; i < 300; i++);
//        delay_ms(1);
        LATBbits.LATB9 = 0;
        for(i = 0; i < 300; i++);
//        delay_ms(1);
    }
    
    return 1;
}