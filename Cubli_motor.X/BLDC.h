#ifndef _BLDC_H
#define _BLDC_H

extern inline void MotorPhase(unsigned char, unsigned int);
extern inline unsigned char GetHallPhase();

extern volatile unsigned char current_phase;
extern volatile signed char commutate;
extern volatile unsigned int motor_speed;

#endif