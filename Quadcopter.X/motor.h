#ifndef _MOTOR_H_
#define _MOTOR_H_

#define MOTOR_OFF 1924
#define MOTOR_MAX 3845

typedef struct{
    int upRight, downRight, downLeft, upLeft;
} Motors;

/*
 * 
 *  |upLeft|       |upRight|
 *      \             / 
 *        \    ?    / 
 *          \ ___ /
 *           |   |
 *           |___|
 *          /     \
 *        /         \
 *      /             \ 
 * |downLeft|      |downRight|
 *  
 */

extern void LimitSpeed(Motors*);
extern void MotorsReset(Motors*);
extern void CalibrateESC();
extern void WriteMotors(Motors);
extern void TurnMotorsOff();

#endif