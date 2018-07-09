#ifndef _XBee_H_
#define _XBee_H_

#include "PID.h"
#include <stdbool.h>

volatile int remote_x1 = 0, remote_y1 = 0, remote_x2 = 0, remote_y2 = 0;
volatile int safety_counter = 0;

volatile unsigned char dial1, dial2;
volatile unsigned char receive1;
volatile unsigned char tx_buffer_index = 0;
volatile bool tx_flag = 0;

volatile bool left_switch = 0, right_switch = 0;
volatile bool Xbee_signal = 0;

char tx_buffer[200];

extern void SendCalibrationData();
extern void SendFlightData(PID, PID, PID, PID, char);

#endif