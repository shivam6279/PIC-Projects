#ifndef _FUNCS_H
#define _FUNCS_H

#include "PID.h"
#include "MPU6050.h"

#define UART_XBee 1
#define UART_A 4
#define UART_B 2
#define UART_C 5
#define ACC_LOOP_TIME 2000

extern char xbee_mode;
extern bool run_motor;

extern signed int parse_rx_int();

extern unsigned char get_face(float, float);
extern unsigned char get_edge(float, float, float);
extern unsigned char get_corner(float, float, float);

extern void balance_edge(float, float, float, XYZ, PID*, PID*, PID*);
extern void balance_corner(float[4], float, XYZ);

extern void TransmitDebug(float, float, float, unsigned char, unsigned char, XYZ, PID, PID, PID);

#endif