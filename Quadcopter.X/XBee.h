#ifndef _XBee_H_
#define _XBee_H_

#include "PID.h"
#include <stdbool.h>

#define XBEE_START_BYTE 0x40
#define XBEE_END_BYTE	0x80

typedef struct {
	int x1, y1, x2, y2;
	unsigned char d1, d2;
	bool ls, rs;
	bool signal;
    bool data_ready;
} rx;

extern volatile rx XBee, XBee_temp;

extern volatile int safety_counter;

extern volatile unsigned char tx_buffer_timer = 0;
extern volatile unsigned char tx_buffer_index;
extern volatile bool tx_flag;
extern volatile bool XBee_signal_temp;

extern char tx_buffer[200];

extern void XBeeReset();
extern rx ReadXBee();
extern void SendCalibrationData();
extern void SendFlightData(PID, PID, PID, PID, char);

#endif