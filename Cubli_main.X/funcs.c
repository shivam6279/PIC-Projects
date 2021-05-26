#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "funcs.h"
#include "USART.h"
#include "pic32.h"
#include "XBee.h"
#include "AHRS.h"
#include "PID.h"
#include "MPU6050.h"

#define get_angle_diff(a, b) fabs(fabs(fmod(a-b+180.0, 360.0)) - 180.0)

char xbee_mode = 'S';
bool run_motor = true;

float edge_centerpoints[12][2] = {{44.0, -90}, {41.8, 180}, {49.0, 90}, {51.7, 0}, {-40.3, -90}, {-40.0, 180}, {-46.0, 90}, {-47.0, 0}, {0.0, -135.8}, {0.0, 142.0}, {0.0, 47.0}, {0.0, -51.7}};
float edge_setpoints[12] = {-44.0, };
unsigned char edge_pitch_or_roll[12] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1};
int edge_angle_signs[12] = {-1, 1, 1, -1, -1, 1, 1, -1, -1, -1, -1, -1};
unsigned char edge_motor[12] = {2, 1, 2, 1, 2, 1, 2, 1, 0, 0, 0, 0};

unsigned char get_face(float pitch_angle, float roll_angle) {
    float offset = 15.0;
    if(fabs(pitch_angle - 90.0) < offset) {
        return 1;
    } else if(fabs(pitch_angle - (-90.0)) < offset) {
        return 6;
    } else if(fabs(roll_angle - (-90.0)) < offset){
        return 2;
    } else if(fabs(180.0 - fabs(roll_angle)) < offset){
        return 3;
    } else if(fabs(roll_angle - 0.0) < offset){
        return 4;
    } else if(fabs(roll_angle - 90.0) < offset){
        return 5;
    }
    return 0;
}

unsigned char get_edge(float pitch_angle, float roll_angle, float offset) {
    unsigned char i, index;
    float min = 360, t = 370.0;
    
    for(i = 0, index = 0; i < 12; i++) {
        if(fabs(fmod(edge_centerpoints[i][0], 90.0)) < 15 && get_angle_diff(edge_centerpoints[i][0], pitch_angle) < 30) {
            t = fabs(roll_angle - edge_centerpoints[i][1]);
        } else if(fabs(fmod(edge_centerpoints[i][1], 90.0)) < 15 && get_angle_diff(edge_centerpoints[i][1], roll_angle) < 30) {
            t = fabs(pitch_angle - edge_centerpoints[i][0]);
        }
        
        if(t < min) {
            min = t;
            index = i;
        }
    }
    
    if(min < offset) {
        return index + 1;
    }
    
    return 0;
}

void balance_edge(float pitch_angle, float roll_angle, float loop_time, XYZ gyro, PID* motorA, PID* motorB, PID* motorC) {    
    float ks;
    float kv;  
    
    static unsigned char pre_edge = 0, pre_edge2 = 0;
    
    static bool balance = false;
    
    float angle_offset, angle, rpm;
    unsigned char port;
    
    float temp;
    
    PID *motor;
    
    unsigned char edge = get_edge(pitch_angle, roll_angle, 1.0);
    unsigned char edge2 = get_edge(pitch_angle, roll_angle, 20.0);
    
    if(!edge2) {    
        if(!edge2 && pre_edge2) {
            balance = false;

            USART_write_int(UART_A, 0);
            USART_send(UART_A, '\r');
            USART_write_int(UART_B, 0);
            USART_send(UART_B, '\r');
            USART_write_int(UART_C, 0);
            USART_send(UART_C, '\r');

            USART_send_str(UART_A, "OO\r");
            USART_send_str(UART_B, "OO\r");
            USART_send_str(UART_C, "OO\r");

            motorA->integral = 0.0;
            motorB->integral = 0.0;
            motorC->integral = 0.0;        
        }
    } else {    
        if(edge_motor[edge2-1] == 0) {
            port = UART_A;
            motor = motorA;        
        } else if(edge_motor[edge2-1] == 1) {
            port = UART_B;
            motor = motorB;        
        } else {
            port = UART_C;
            motor = motorC;
        }        
        angle_offset = edge_centerpoints[edge2-1][edge_pitch_or_roll[edge2-1]]; 
        
        if(edge_pitch_or_roll[edge2-1]) 
            angle = roll_angle;
        else
            angle = pitch_angle;
        
        if(edge && !pre_edge) {
            balance = true;
            USART_send_str(port, "I\r");
            motor->offset = angle_offset;
        }    
        
        if(balance) {            
            motor->error = edge_angle_signs[edge2-1] * (angle - motor->offset);

            if(edge_motor[edge2-1] == 0) {
                rpm = rpm_A;
                motor->derivative = -gyro.x;        
                ks = 0.3;
                kv = 0.5;

            } else if(edge_motor[edge2-1] == 1) {
                rpm = rpm_B;
                motor->derivative = -gyro.y;        
                ks = 0.3;
                kv = 0.5;

            } else {
                rpm = rpm_C;
                motor->derivative = -gyro.z;        
                ks = 0.3;
                kv = 0.5;
            }

            PIDIntegrate(motor, loop_time);
            motor->offset += edge_angle_signs[edge2-1] * ks * motor->error * loop_time;
            motor->offset = LimitValue(motor->offset, angle_offset-10.0, angle_offset+10.0);   
            PIDOutput(motor);            
            motor->output += kv*rpm; 
            motor->output =  LimitValue(motor->output, -1000.0, 1000.0);  

            if(run_motor) {
                USART_write_int(port, motor->output);
                USART_send(port, '\r');                
            } 
        }
    }
    pre_edge = edge;
    pre_edge2 = edge2;
}

void Corner(float q[4]) {
    PID roll, pitch;
    float pitch_angle = 0.0, roll_angle = 0.0, heading;
    XYZ acc, gyro, gyro_global, gyro_filt; 
    float acc_loop_time;  
    float gyro_spin = 0, gyro_mag, gyro_mag_sp;
    
    float A_out, B_out, C_out;
    
    float sx, sy, cx, cy;
    
    float ks_a = 0.8;
    float ks_b = 0.8;
    float ks_c = 0.8;
    
    float kv = 0.2;
    
    XYZ ra, rb, rc;
    
    PIDSet(&roll, 150, 0, 35);
    PIDSet(&pitch, 150, 0, 35);
    roll.offset = 47.0;
    pitch.offset = -36.7;
    
    float yaw_angle = 0.0, yaw_output;
    
    esc_counter = 0;
    
    while(1) {
        if(esc_counter > 15000000 && gyro_spin == 0) {
            gyro_spin = 25.0;
            esc_counter = 0;
        }
        
        if(acc_aq_counter >= ACC_LOOP_TIME) {
            acc_loop_time = (double)acc_aq_counter / 1000000.0;  
            acc_aq_counter = 0;
            
            if(mode == 'R') {
                run_motor = true;
            } else if(mode == 'O') {
                run_motor = false;
            } else {
                xbee_mode = mode;
            }

            GetGyro(&gyro);
            GetAcc(&acc);
            
            gyro_filt.x = 0.85 * gyro_filt.x + 0.15 * gyro.x;
            gyro_filt.y = 0.85 * gyro_filt.y + 0.15 * gyro.y;
            gyro_filt.z = 0.85 * gyro_filt.z + 0.15 * gyro.z;
            
            MadgwickQuaternionUpdateGyro(q, gyro, acc_loop_time);
            MadgwickQuaternionUpdateAcc(q, acc, acc_loop_time);
            QuaternionToEuler(q, &roll_angle, &pitch_angle, &heading);
            gyro_global = MultiplyVectorQuaternion(gyro_filt, q);
            
            cx = cos(pitch_angle * M_PI / 180.0);
            sx = sin(pitch_angle * M_PI / 180.0);
            cy = cos((90.0 - roll_angle) * M_PI / 180.0);
            sy = sin((90.0 - roll_angle) * M_PI / 180.0);
            
            rc.x = fabs(cy);
            rc.y = fabs(sx*sy);
            rc.z = fabs(cx*sy);
            ra.x = 0;
            ra.y = fabs(cx);
            ra.z = fabs(sx);
            rb.x = fabs(sy);
            rb.y = fabs(cy*sx);
            rb.z = fabs(cx*cy);
            
            yaw_angle += (gyro_global.z - gyro_spin) * acc_loop_time;
            yaw_output = 25 * (gyro_global.z - gyro_spin) + 30.0 * yaw_angle;
            
            if(fabs(-36.7 - pitch_angle) > 20 || fabs(46.5 - roll_angle) > 20)
                return;
            
            roll.error = -(roll_angle - roll.offset);         
            PIDIntegrate(&roll, acc_loop_time);
            roll.offset -= kv * roll.error * acc_loop_time;
            roll.offset = LimitValue(roll.offset, 36, 56);
            
            pitch.error = pitch_angle - pitch.offset;    
            PIDIntegrate(&pitch, acc_loop_time);
            pitch.offset += kv * pitch.error * acc_loop_time;
            pitch.offset = LimitValue(pitch.offset, -46, -26);  
            
            A_out = 150.0 * ra.y*roll.error;
            A_out += 30.0 * -gyro_filt.x;
            A_out += ra.z*yaw_output;
            A_out += ks_a*rpm_A;
            A_out = LimitValue(A_out, -1000.0, 1000.0);

            B_out = 200.0 * rb.x*-pitch.error;
            B_out += 150.0 * rb.y*-roll.error;
            B_out += 30.0 * -gyro_filt.y;
            B_out += rb.z*yaw_output;
            B_out += ks_b*rpm_B;
            B_out = LimitValue(B_out, -1000.0, 1000.0);

            C_out = 200.0 * rc.x*pitch.error;
            C_out += 150.0 * rc.y*-roll.error;
            C_out += 30.0 * -gyro_filt.z;
            C_out += rc.z*yaw_output;
            C_out += ks_c*rpm_C;
            C_out = LimitValue(C_out, -1000.0, 1000.0);

            if(run_motor) {
                USART_write_int(UART_A, A_out);
                USART_send(UART_A, '\r');   
                USART_write_int(UART_B, B_out);
                USART_send(UART_B, '\r');  
                USART_write_int(UART_C, C_out);
                USART_send(UART_C, '\r');  
            }
        }
           
        if(ms_counter() > 25) {
            StartDelayCounter();
//            USART_write_float(1, pitch_angle, 4);     
//            USART_send_str(UART_XBee, ", ");
//            USART_write_float(1, pitch.offset, 4);     
//            USART_send_str(UART_XBee, ", ");
//            USART_write_float(1, roll_angle, 4);
//            USART_send_str(UART_XBee, ", ");
//            USART_write_float(1, roll.offset, 4);
//            USART_send_str(UART_XBee, "\n");
            
            USART_write_float(1, gyro_filt.x, 4);     
            USART_send_str(UART_XBee, ", ");
            USART_write_float(1, gyro_filt.y, 4);     
            USART_send_str(UART_XBee, ", ");
            USART_write_float(1, gyro_filt.z, 4);
            USART_send_str(UART_XBee, ", ");
            USART_write_float(1, gyro_global.z, 4);
            USART_send_str(UART_XBee, "\n");
        }
    }
}

void TransmitDebug(float pitch_angle, float roll_angle, float heading, unsigned char face, unsigned char edge, XYZ gyro, PID motorA, PID motorB, PID motorC) {
    if(xbee_mode == 'A') {
        USART_write_float(UART_XBee, pitch_angle, 4);     
        USART_send_str(UART_XBee, ", ");
        USART_write_float(UART_XBee, roll_angle, 4);
        USART_send_str(UART_XBee, ", ");
        USART_write_float(UART_XBee, heading, 4);
        USART_send_str(UART_XBee, "\n");

    } else if(xbee_mode == 'E') {
        USART_write_int(UART_XBee, face);     
        USART_send_str(UART_XBee, ", ");
        USART_write_int(UART_XBee, edge); 
        USART_send_str(UART_XBee, "\n");

    } else if(xbee_mode == 'G') {
        USART_write_float(UART_XBee, gyro.x, 4);     
        USART_send_str(UART_XBee, ", ");
        USART_write_float(UART_XBee, gyro.y, 4);
        USART_send_str(UART_XBee, ", ");
        USART_write_float(UART_XBee, gyro.z, 4);
        USART_send_str(UART_XBee, "\n");

    } else if(xbee_mode == 'V') {
        USART_write_float(UART_XBee, rpm_A, 4);     
        USART_send_str(UART_XBee, ", ");
        USART_write_float(UART_XBee, rpm_B, 4);
        USART_send_str(UART_XBee, ", ");
        USART_write_float(UART_XBee, rpm_C, 4);
        USART_send_str(UART_XBee, "\n");

    }  else if(xbee_mode == 'P') {
        if(edge == 9 || edge == 10 || edge == 11 || edge == 12) {
            USART_write_float(UART_XBee, motorA.error*motorA.kp, 4);    
            USART_send_str(UART_XBee, ", ");
            USART_write_float(UART_XBee, motorA.integral*motorA.ki, 4);
            USART_send_str(UART_XBee, ", ");                    
            USART_write_float(UART_XBee, motorA.derivative*motorA.kd, 4);
            USART_send_str(UART_XBee, ", ");
            USART_write_float(UART_XBee, motorA.output, 4);
            USART_send_str(UART_XBee, "\n");
        } else if(edge == 2 || edge == 4 || edge == 6 || edge == 8) {
            USART_write_float(UART_XBee, motorB.error*motorB.kp, 4);    
            USART_send_str(UART_XBee, ", ");
            USART_write_float(UART_XBee, motorB.integral*motorB.ki, 4);
            USART_send_str(UART_XBee, ", ");                    
            USART_write_float(UART_XBee, motorB.derivative*motorB.kd, 4);
            USART_send_str(UART_XBee, ", ");
            USART_write_float(UART_XBee, motorB.output, 4);
            USART_send_str(UART_XBee, "\n");
        } else if(edge == 1 || edge == 3 || edge == 5 || edge == 7) {
            USART_write_float(UART_XBee, motorC.error*motorC.kp, 4);    
            USART_send_str(UART_XBee, ", ");
            USART_write_float(UART_XBee, motorC.integral*motorC.ki, 4);
            USART_send_str(UART_XBee, ", ");                    
            USART_write_float(UART_XBee, motorC.derivative*motorC.kd, 4);
            USART_send_str(UART_XBee, ", ");
            USART_write_float(UART_XBee, motorC.output, 4);
            USART_send_str(UART_XBee, "\n");
        } else {
            USART_write_float(UART_XBee, pitch_angle, 4);     
            USART_send_str(UART_XBee, ", ");
            USART_write_float(UART_XBee, roll_angle, 4);
            USART_send_str(UART_XBee, "\n");
        }

    } else {
        if(edge == 9 || edge == 10 || edge == 11 || edge == 12) {
            USART_write_float(UART_XBee, roll_angle, 4);     
            USART_send_str(UART_XBee, ", ");
            USART_write_float(UART_XBee, motorA.offset, 4);
            USART_send_str(UART_XBee, "\n");
        } else if(edge == 2 || edge == 4 || edge == 6 || edge == 8) {
            USART_write_float(UART_XBee, pitch_angle, 4);     
            USART_send_str(UART_XBee, ", ");
            USART_write_float(UART_XBee, motorB.offset, 4);
            USART_send_str(UART_XBee, "\n");
        } else if(edge == 1 || edge == 3 || edge == 5 || edge == 7) {
            USART_write_float(UART_XBee, pitch_angle, 4);     
            USART_send_str(UART_XBee, ", ");
            USART_write_float(UART_XBee, motorC.offset, 4);
            USART_send_str(UART_XBee, "\n");
        } else {
            USART_write_float(UART_XBee, pitch_angle, 4);     
            USART_send_str(UART_XBee, ", ");
            USART_write_float(UART_XBee, roll_angle, 4);
            USART_send_str(UART_XBee, "\n");
        }
    } 
}