#ifndef _XBee_H_
#define _XBee_H_

#include "PID.h"
#include <stdbool.h>

extern volatile int remote_x1, remote_y1, remote_x2, remote_y2;
extern volatile int safety_counter;

extern volatile unsigned char dial1, dial2;
extern volatile unsigned char receive1;
extern volatile unsigned char tx_buffer_index;
extern volatile bool tx_flag;

extern volatile bool left_switch, right_switch;
extern volatile bool Xbee_signal;

extern char tx_buffer[200];

extern void SendCalibrationData();
extern void SendFlightData(PID, PID, PID, PID, char);

#endif