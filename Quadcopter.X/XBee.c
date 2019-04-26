#include <xc.h>
#include "XBee.h"
#include "pic32.h"
#include "10DOF.h"
#include "PID.h"
#include "GPS.h"
#include "USART.h"
#include "EEPROM.h"

volatile rx XBee, XBee_temp;

volatile int safety_counter = 0;
volatile int tx_buffer_index = 0;
volatile bool XBee_signal_temp = 0;

volatile char tx_buffer[XBEE_TX_BUFFER_LEN];

void XBeeReset() {
    XBee.x1 = 0;
    XBee.y1 = 0;
    XBee.x2 = 0;
    XBee.y2 = 0;
    XBee.ls = 0;
    XBee.rs = 0;
    XBee.signal = 0;
    XBee.data_ready = 0;
    XBee_signal_temp = 0;
}

rx ReadXBee() {
    return XBee;
}

void SendCalibrationData() {  
    const float avg_factor = 25;

    XYZ compass_min, compass_max;
    XYZ compass_avg;

    GetRawIMU();

    compass_max = compass;    
    compass_min = compass; 
    compass_avg = compass;
    
    TX_TIMER_ON = 1;
    tx_buffer_index = 0;
    while(XBee.rs == 0) {
        GetRawIMU();

        compass_avg.x = ((avg_factor - 1) * compass_avg.x + compass.x) / avg_factor;
        compass_avg.y = ((avg_factor - 1) * compass_avg.y + compass.y) / avg_factor;
        compass_avg.z = ((avg_factor - 1) * compass_avg.z + compass.z) / avg_factor;

        if(compass_avg.x > compass_max.x) compass_max.x = compass_avg.x;
        if(compass_avg.y > compass_max.y) compass_max.y = compass_avg.y;
        if(compass_avg.z > compass_max.z) compass_max.z = compass_avg.z;
        
        if(compass_avg.x < compass_min.x) compass_min.x = compass_avg.x;
        if(compass_avg.y < compass_min.y) compass_min.y = compass_avg.y;
        if(compass_avg.z < compass_min.z) compass_min.z = compass_avg.z;
        
        tx_buffer[0] = 'D';
        StrWriteInt(compass.x, tx_buffer, 1);
        StrWriteInt(compass.y, tx_buffer, 8);
        StrWriteInt(compass.z, tx_buffer, 15);
        StrWriteInt(compass_min.x, tx_buffer, 22);
        StrWriteInt(compass_min.y, tx_buffer, 29);
        StrWriteInt(compass_min.z, tx_buffer, 36);
        StrWriteInt(compass_max.x, tx_buffer, 43);
        StrWriteInt(compass_max.y, tx_buffer, 50);
        StrWriteInt(compass_max.z, tx_buffer, 57);
        StrWriteInt(gyro.x, tx_buffer, 64);
        StrWriteInt(gyro.y, tx_buffer, 71);
        StrWriteInt(gyro.z, tx_buffer, 78);
        StrWriteInt(acc.x, tx_buffer, 85);
        StrWriteInt(acc.y, tx_buffer, 92);
        StrWriteInt(acc.z, tx_buffer, 99);
        tx_buffer[106] = '\r';
        tx_buffer[107] = '\0';
    
        //tx_flag = 1;
        delay_ms(35);
    }

    ComputeCompassOffsetGain(compass_min, compass_max);

#if board_version == 4
    eeprom_writeCalibration();
#endif
}

void XBeeFillBuffer() {
    int i;
    while(!U1STAbits.UTXBF && tx_buffer_index > 0) {
        U1TXREG = tx_buffer[0];
        for(i = 0; i < tx_buffer_index; i++)
            tx_buffer[i] = tx_buffer[i + 1];
        tx_buffer_index--;
    }
    if(tx_buffer_index) {
        IFS3bits.U1TXIF = 0;
        UART1_TX_INTERRUPT = 1;
    }
}

void XBeeClearBuffer() {
    UART1_TX_INTERRUPT = 0;
    tx_buffer[0] = '\0';
    tx_buffer_index = 0;
}

void XBeeWriteInt(int a) {
    long int tens;

    UART1_TX_INTERRUPT = 0;
    
    if(a < 0) { 
        a *= -1; 
        tx_buffer[tx_buffer_index++] = '-'; 
    }
    
    if(a > 1) {
        for(tens = 1; tens < a; tens *= 10);
        tens /= 10;
    } else {
        tens = 1;
    }

    for(; tens > 0; tens /= 10)
        tx_buffer[tx_buffer_index++] = ((long int)(a / tens) % 10) + 48;

    tx_buffer[tx_buffer_index] = '\0';

    XBeeFillBuffer();
}

void XBeeWriteFloat(float a, unsigned char precision) {
    unsigned char i;
    long int tens;

    UART1_TX_INTERRUPT = 0;
    
    if(a < 0) { 
        a *= -1; 
        tx_buffer[tx_buffer_index++] = '-'; 
    }
    
    if(a > 1.0) {
        for(tens = 1; tens < a; tens *= 10);
        tens /= 10;

        for(; tens > 0; tens /= 10)
            tx_buffer[tx_buffer_index++] = ((long int)(a / tens) % 10) + 48;
    } else {
        tx_buffer[tx_buffer_index++] = '0';
    }

    tx_buffer[tx_buffer_index++] = '.';
    for(i = 0, tens = 10; i < precision; i++, tens *= 10, tx_buffer_index++)
        tx_buffer[tx_buffer_index] = ((long int)(a * tens) % 10) + 48;

    tx_buffer[tx_buffer_index] = '\0';

    XBeeFillBuffer();
}

void XBeeWriteChar(char a) {
    UART1_TX_INTERRUPT = 0;

    tx_buffer[tx_buffer_index++] = a;
    tx_buffer[tx_buffer_index] = '\0';

    XBeeFillBuffer();
}

void XBeeWriteStr(const char str[]) {
    unsigned char i;

    UART1_TX_INTERRUPT = 0;
    
    for(i = 0; str[i] != '\0'; i++, tx_buffer_index++) 
        tx_buffer[tx_buffer_index] = str[i];

    tx_buffer[tx_buffer_index] = '\0';

    XBeeFillBuffer();
}

void XBeeWriteRawInt(int a) {
    char str[2];

    UART1_TX_INTERRUPT = 0;

    *(int*)(tx_buffer + tx_buffer_index) = a;
    tx_buffer_index += 2;

    tx_buffer[tx_buffer_index] = '\0';

    XBeeFillBuffer();
}

void XBeeWriteRawFloat(float a) {
    char str[4];

    UART1_TX_INTERRUPT = 0;

    *(float*)(tx_buffer + tx_buffer_index) = a;
    tx_buffer_index += 4;

    tx_buffer[tx_buffer_index] = '\0';

    XBeeFillBuffer();
}
