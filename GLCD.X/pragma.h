#ifndef _pragma_H_
#define _pragma_H_

// Device Config Bits in  DEVCFG1:		
#pragma config FNOSC = SPLL	
#pragma config FSOSCEN = OFF	
//#pragma config FWDTEN = OFF  


#pragma config POSCMOD = OFF
#pragma config OSCIOFNC = ON

// Device Config Bits in  DEVCFG2:
#pragma config FPLLICLK = PLL_FRC
#pragma config FPLLIDIV = DIV_1
#pragma config FPLLMULT = MUL_50
#pragma config FPLLODIV = DIV_2
#pragma config FPLLRNG = RANGE_5_10_MHZ
#pragma config FWDTEN = OFF
#pragma config FDMTEN = OFF

/*#pragma config POSCMOD = EC
#pragma config OSCIOFNC = OFF

#pragma config FPLLICLK = PLL_POSC
#pragma config FPLLIDIV = DIV_3
#pragma config FPLLMULT = MUL_50
#pragma config FPLLODIV = DIV_2
#pragma config FPLLRNG = RANGE_5_10_MHZ
#pragma config FWDTEN = OFF
#pragma config FDMTEN = OFF*/

#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config TRCEN = ON               // Trace Enable (Trace features in the CPU are enabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = ALLOW_PG2       // Debug Mode CPU Access Permission (Allow CPU access to Permission Group 2 permission regions)
#pragma config EJTAGBEN = NORMAL  

#pragma config PGL1WAY = OFF
#pragma config PMDL1WAY = OFF
#pragma config IOL1WAY = OFF

#endif