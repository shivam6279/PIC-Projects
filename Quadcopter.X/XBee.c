#include <xc.h>
#include "XBee.h"
#include "pic32.h"
#include "10DOF.h"
#include "PID.h"
#include "GPS.h"
#include "USART.h"
#include "EEPROM.h"
#include <string.h>
#include <sys/attribs.h>

volatile rx XBee;

volatile int safety_counter = 0;

static volatile int tx_buffer_index = 0;
static volatile bool XBee_signal_temp = 0;

static volatile char tx_buffer[XBEE_TX_BUFFER_LEN];

void __ISR_AT_VECTOR(_UART1_RX_VECTOR, IPL6SRS) XBee_rx(void) {
    IFS3bits.U1RXIF = 0; 

    static unsigned char XBee_rx_byte, XBee_address;
    static rx XBee_temp;

    do {
        XBee_rx_byte = U1RXREG & 0xFF;
        XBee_address = XBee_rx_byte >> 5;

        switch(XBee_address) {
            case 0:
                XBee_temp.x1 = (XBee_rx_byte & 0x1F) - 15;
                XBee_signal_temp = 1;
                break;

            case 1:
                XBee_temp.y1 = (XBee_rx_byte & 0x1F) - 15;
                break;

            case 2:
                XBee_temp.x2 = (XBee_rx_byte & 0x1F) - 15;
                break;

            case 3:
                XBee_temp.y2 = (XBee_rx_byte & 0x1F);
                break;

            case 4:
                XBee_temp.ls = (XBee_rx_byte >> 1) & 1; 
                XBee_temp.rs = XBee_rx_byte & 1;
                break;

            case 5: 
                XBee_temp.d2 = (XBee_rx_byte & 0b00001100) << 2;
                XBee_temp.d1 = (XBee_rx_byte & 0b00000011);
                safety_counter = 0;
                if(XBee_signal_temp) {
                    XBee_signal_temp = 0;
                    XBee_temp.signal = 1;
                    XBee_temp.data_ready = 1;

                    XBee = XBee_temp;
                }
                break;
        }
    }while(U1STAbits.URXDA);
    
    IFS3bits.U1RXIF = 0; 
}

void __ISR_AT_VECTOR(_UART1_TX_VECTOR, IPL6SRS) XBee_tx(void) {
    static int i;

    IFS3bits.U1TXIF = 0;

    if(tx_buffer_index && !U1STAbits.UTXBF) {
        U1TXREG = tx_buffer[0];
        if(tx_buffer_index == 1) {
            tx_buffer_index = 0;
            UART1_TX_INTERRUPT = 0;
        } else {
            for(i = 0; i < tx_buffer_index; i++)
                tx_buffer[i] = tx_buffer[i + 1];
            tx_buffer_index--;
        }
    }
}

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
    
    XYZ acc, gyro, compass;
    XYZ compass_min, compass_max, compass_avg;

    GetRawAcc(&acc);
    GetRawGyro(&gyro);
    GetRawCompass(&compass);

    compass_max = compass;    
    compass_min = compass; 
    compass_avg = compass;
    
    tx_buffer_index = 0;
    
    bool p_ls = XBee.ls;
    while(XBee.ls == p_ls) {
        GetRawAcc(&acc);
        GetRawGyro(&gyro);
        GetRawCompass(&compass);

        compass_avg.x = ((avg_factor - 1) * compass_avg.x + compass.x) / avg_factor;
        compass_avg.y = ((avg_factor - 1) * compass_avg.y + compass.y) / avg_factor;
        compass_avg.z = ((avg_factor - 1) * compass_avg.z + compass.z) / avg_factor;

        if(compass_avg.x > compass_max.x) compass_max.x = compass_avg.x;
        if(compass_avg.y > compass_max.y) compass_max.y = compass_avg.y;
        if(compass_avg.z > compass_max.z) compass_max.z = compass_avg.z;
        
        if(compass_avg.x < compass_min.x) compass_min.x = compass_avg.x;
        if(compass_avg.y < compass_min.y) compass_min.y = compass_avg.y;
        if(compass_avg.z < compass_min.z) compass_min.z = compass_avg.z;
        
        XBeePacketChar('D');        
        XBeePacketInt(acc.x); XBeePacketChar(',');
        XBeePacketInt(acc.y); XBeePacketChar(',');
        XBeePacketInt(acc.z); XBeePacketChar(',');
        XBeePacketInt(gyro.x); XBeePacketChar(',');
        XBeePacketInt(gyro.y); XBeePacketChar(',');
        XBeePacketInt(gyro.z); XBeePacketChar(',');        
        XBeePacketInt(compass.x); XBeePacketChar(',');
        XBeePacketInt(compass.y); XBeePacketChar(',');
        XBeePacketInt(compass.z); XBeePacketChar(',');
        XBeePacketInt(compass_max.x); XBeePacketChar(',');
        XBeePacketInt(compass_max.y); XBeePacketChar(',');
        XBeePacketInt(compass_max.z); XBeePacketChar(',');
        XBeePacketInt(compass_min.x); XBeePacketChar(',');
        XBeePacketInt(compass_min.y); XBeePacketChar(',');
        XBeePacketInt(compass_min.z);
        XBeePacketSend();
    
        delay_ms(35);
    }
    ComputeCompassOffsetGain(compass_min, compass_max);
#if (board_version == 4 || board_version == 5) && USE_EEPROM == 1
    eeprom_writeCalibration(compass_min, compass_max);
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

bool TxBufferEmpty() {
    if(tx_buffer_index)
        return 0;
    return 1;
}

void XBeeWriteChar(char a) {
    UART1_TX_INTERRUPT = 0;

    tx_buffer[tx_buffer_index++] = a;
    tx_buffer[tx_buffer_index] = '\0';
    
    XBeeFillBuffer();
}

void XBeeWriteStr(const char str[]) {
    unsigned int i;

    UART1_TX_INTERRUPT = 0;
    
    for(i = 0; str[i] != '\0'; i++, tx_buffer_index++) 
        tx_buffer[tx_buffer_index] = str[i];

    tx_buffer[tx_buffer_index] = '\0';
    
    XBeeFillBuffer();
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

unsigned char XBeeWriteFloat(float a, unsigned char precision) {
    unsigned char i, len = 0;;
    long int tens;

    UART1_TX_INTERRUPT = 0;
    
    if(a < 0) { 
        a *= -1; 
        tx_buffer[tx_buffer_index++] = '-'; 
        len++;
    }
    
    if(a > 1.0) {
        for(tens = 1; tens < a; tens *= 10);
        tens /= 10;

        for(; tens > 0; tens /= 10, len++)
            tx_buffer[tx_buffer_index++] = ((long int)(a / tens) % 10) + 48;
    } else {
        tx_buffer[tx_buffer_index++] = '0';
        len++;
    }

    tx_buffer[tx_buffer_index++] = '.';
    len++;
    
    for(i = 0, tens = 10; i < precision; i++, tens *= 10, len++)
        tx_buffer[tx_buffer_index++] = ((long int)(a * tens) % 10) + 48;

    tx_buffer[tx_buffer_index] = '\0';

    XBeeFillBuffer();
    
    return len;
}

unsigned char FloatStrLen(float a, unsigned char precision) {
    unsigned char i, len = 0;;
    long int tens;
    
    if(a < 0) { 
        a *= -1;
        len++;
    }
    
    if(a > 1.0) {
        for(tens = 1; tens < a; tens *= 10);
        tens /= 10;

        for(; tens > 0; tens /= 10, len++);
    } else {
        len++;
    }
    len++;
    
    for(i = 0, tens = 10; i < precision; i++, tens *= 10, len++);
    
    return len;
}

void XBeeWriteRawInt(int a) {
    UART1_TX_INTERRUPT = 0;

    *(int*)(tx_buffer + tx_buffer_index) = a;
    tx_buffer_index += 2;

    tx_buffer[tx_buffer_index] = '\0';

    XBeeFillBuffer();
}

void XBeeWriteRawFloat(float a) {
    UART1_TX_INTERRUPT = 0;

    *(float*)(tx_buffer + tx_buffer_index) = a;
    tx_buffer_index += 4;

    tx_buffer[tx_buffer_index] = '\0';

    XBeeFillBuffer();
}


void XBeePacketSend() {
    int i;
    tx_buffer_index += 4;
    
    UART1_TX_INTERRUPT = 0;
    
    for(i = tx_buffer_index; i >= 4; i--) {
        tx_buffer[i] = tx_buffer[i-4];
    }
    tx_buffer[0] = '\f';    
    tx_buffer[1] = (((tx_buffer_index - 4) / 100) + '0');
    tx_buffer[2] = (((tx_buffer_index - 4) / 10) % 10 + '0');
    tx_buffer[3] = ((tx_buffer_index - 4) % 10 + '0');
    
    XBeeFillBuffer();
}

void XBeePacketChar(char a) {
    UART1_TX_INTERRUPT = 0;

    tx_buffer[tx_buffer_index++] = a;
    tx_buffer[tx_buffer_index] = '\0';
}

void XBeePacketStr(const char str[]) {
    unsigned int i;

    UART1_TX_INTERRUPT = 0;
    
    for(i = 0; str[i] != '\0'; i++, tx_buffer_index++) 
        tx_buffer[tx_buffer_index] = str[i];

    tx_buffer[tx_buffer_index] = '\0';
}

void XBeePacketInt(int a) {
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
}

unsigned char XBeePacketFloat(float a, unsigned char precision) {
    unsigned char i, len = 0;;
    long int tens;

    UART1_TX_INTERRUPT = 0;
    
    if(a < 0) { 
        a *= -1; 
        tx_buffer[tx_buffer_index++] = '-'; 
        len++;
    }
    
    if(a > 1.0) {
        for(tens = 1; tens < a; tens *= 10);
        tens /= 10;

        for(; tens > 0; tens /= 10, len++)
            tx_buffer[tx_buffer_index++] = ((long int)(a / tens) % 10) + 48;
    } else {
        tx_buffer[tx_buffer_index++] = '0';
        len++;
    }

    tx_buffer[tx_buffer_index++] = '.';
    len++;
    
    for(i = 0, tens = 10; i < precision; i++, tens *= 10, len++)
        tx_buffer[tx_buffer_index++] = ((long int)(a * tens) % 10) + 48;

    tx_buffer[tx_buffer_index] = '\0';
    
    return len;
}

unsigned char XBeePacketFixedFloat(float a, unsigned char left, unsigned char precision) {
    unsigned char i, len = 0;;
    long int tens;

    UART1_TX_INTERRUPT = 0;
    
    if(a < 0) { 
        a *= -1; 
        tx_buffer[tx_buffer_index++] = '-';
        len++;
    } else {
        tx_buffer[tx_buffer_index++] = ' '; 
        len++;        
    }    
    
    for(i = 1, tens = 1; i < left; tens *= 10, i++);
    
    for(; left > 0; left--, tens /= 10) {
        if(tens > (long int)a) {
            tx_buffer[tx_buffer_index++] = '0'; 
        } else {
            tx_buffer[tx_buffer_index++] = ((long int)(a / tens) % 10) + '0';
        }
        len++;
    }

    tx_buffer[tx_buffer_index++] = '.';
    len++;
    
    for(i = 0, tens = 10; i < precision; i++, tens *= 10, len++)
        tx_buffer[tx_buffer_index++] = ((long int)(a * tens) % 10) + 48;

    tx_buffer[tx_buffer_index] = '\0';
    
    return len;
}
