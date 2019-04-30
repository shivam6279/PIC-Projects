#ifndef _XBee_H_
#define _XBee_H_

#include "PID.h"
#include <stdbool.h>

#define XBEE_START_BYTE 0x40
#define XBEE_END_BYTE	0x80

#define XBEE_TX_BUFFER_LEN 1024

typedef struct {
	int x1, y1, x2, y2;
	unsigned char d1, d2;
	bool ls, rs;
	bool signal;
    bool data_ready;
} rx;

extern volatile rx XBee, XBee_temp;
extern volatile int safety_counter;

extern void XBeeReset();
extern rx ReadXBee();
extern void SendCalibrationData();

extern void XBeeFillBuffer();
extern void XBeeClearBuffer();
extern bool TxBufferEmpty();
extern void XBeeWriteInt(int);
extern void XBeeWriteFloat(float, unsigned char);
extern void XBeeWriteChar(char);
extern void XBeeWriteStr(const char[]);
extern void XBeeWriteRawInt(int);
extern void XBeeWriteRawFloat(float);

#endif