#include <xc.h>
#include "pragma.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "pic32.h"
#include "bitbang_I2C.h"
#include "MPU6050.h"
#include "USART.h"
#include "XBee.h"
#include "AHRS.h"
#include "PID.h"
#include "funcs.h"

#define ANTI_WINDUP_MAX_BOUND 100
#define ANTI_WINDUP_MAX_ANGLE 100

#define DER_LPF 0.5

#define A_SETPOINT 45.7
#define B_SETPOINT -45.2
#define C_SETPOINT -45.6

#define GYRO_LPF 0.7

void ResetQuaternion(float q[]){
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
}

void main() {
    PID motorA, motorB, motorC;
    float pitch_angle = 0.0, roll_angle = 0.0, heading;
    XYZ acc, gyro, gyro_filt, gravity;
    rx XBee_rx;    
    unsigned char face = 0, edge = 0;    
    float acc_loop_time;
    
    bool flag = false;
    
    int i, k;
    float q[4];    
    
    //Startup initialization   
    PICInit();
    
//    XBeeReset();
//    USART1_init(111111);    
//    timer7_init(1000.0);    // Safety timer for Xbee - 1kHz
    
    USART1_init(115200);
    USART2_init(250000);  
    USART4_init(250000);  
    USART5_init(250000);  
    
    timer2_init(1000.0);    // Delay timer - 1kHz    
    timer4_init(1000000.0); // Loop timer - 1MHz
//    timer5_init(10.0);      // GPS timer - 10Hz
//    timer6_init(312500.0);  // XBee tx timer - 312.5kHz
    
    delay_ms(200);    
    MPU6050Init();
    
    //Set PID gains
    PIDSet(&motorA, 150, 0, 25);
    PIDSet(&motorB, 150, 0, 25);
    PIDSet(&motorC, 150, 0, 25);
    motorA.offset = A_SETPOINT;
    motorB.offset = B_SETPOINT;
    motorC.offset = C_SETPOINT;
    
    delay_ms(500);
    USART_send_str(UART_A, "T\r");
    USART_send_str(UART_B, "T\r");
    USART_send_str(UART_C, "T\r"); 
    delay_ms(500);
    CalibrateGyro();
    ResetQuaternion(q);    
        
    for(i = 0; i < 2000; i++) {
        StartDelayCounter();
        GetAcc(&acc);
        GetGyro(&gyro);
        MadgwickQuaternionUpdateAcc(q, acc, 0.05);
        MadgwickQuaternionUpdateGyro(q, gyro, 0.001);
        while(ms_counter() < 1);
    }
    
    T4CONbits.ON = 1;
    
    QuaternionToEuler(q, &roll_angle, &pitch_angle, &heading);
    
    USART_write_int(UART_A, 0);
    USART_send(UART_A, '\r');
    USART_write_int(UART_B, 0);
    USART_send(UART_B, '\r');
    USART_write_int(UART_C, 0);
    USART_send(UART_C, '\r');
    
    gyro_filt.x = gyro.x;
    gyro_filt.y = gyro.y;
    gyro_filt.z = gyro.z;
    
    StartDelayCounter();
    while(1) {
        if(mode == 'R') {
            run_motor = true;
        } else if(mode == 'O') {
            run_motor = false;
        } else {
            xbee_mode = mode;
        }
        
        if(acc_aq_counter >= ACC_LOOP_TIME) {
            acc_loop_time = (double)acc_aq_counter / 1000000.0;  
            acc_aq_counter = 0;
            
            GetGyro(&gyro);
            GetAcc(&acc);
            
            gyro_filt.x = GYRO_LPF * gyro_filt.x + (1.0-GYRO_LPF) * gyro.x;
            gyro_filt.y = GYRO_LPF * gyro_filt.y + (1.0-GYRO_LPF) * gyro.y;
            gyro_filt.z = GYRO_LPF * gyro_filt.z + (1.0-GYRO_LPF) * gyro.z;
            
            MadgwickQuaternionUpdateGyro(q, gyro, acc_loop_time);
            MadgwickQuaternionUpdateAcc(q, acc, acc_loop_time);
            QuaternionToEuler(q, &roll_angle, &pitch_angle, &heading);

            face = get_face(pitch_angle, roll_angle);
            edge = get_edge(pitch_angle, roll_angle, 20.0);
            
            balance_edge(pitch_angle, roll_angle, acc_loop_time, gyro_filt, &motorA, &motorB, &motorC);            
            if(fabs(-36.7 - pitch_angle) < 2 && fabs(46.5 - roll_angle) < 2) {
                USART_send_str(UART_A, "I\r");
                USART_send_str(UART_B, "I\r");
                USART_send_str(UART_C, "I\r");  
                Corner(q);
                USART_send_str(UART_A, "O\r");
                USART_send_str(UART_B, "O\r");
                USART_send_str(UART_C, "O\r");  
            } 
        }
                  
        if(ms_counter() > 25) {
            StartDelayCounter();
            TransmitDebug(pitch_angle, roll_angle, heading, face, edge, gyro, motorA, motorB, motorC);           
        }
    }
}
