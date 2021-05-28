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

static float edge_centerpoints[12][2] = {{44.0, -90}, {41.8, 180}, {49.0, 90}, {51.7, 0}, {-40.3, -90}, {-40.0, 180}, {-46.0, 90}, {-47.0, 0}, {0.0, -135.8}, {0.0, 142.0}, {0.0, 47.0}, {0.0, -51.7}};
static unsigned char edge_pitch_or_roll[12] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1};
static int edge_angle_signs[12] = {-1, 1, 1, -1, -1, 1, 1, -1, -1, -1, -1, -1};
static unsigned char edge_motor[12] = {2, 1, 2, 1, 2, 1, 2, 1, 0, 0, 0, 0};

float corner_centerpoints[8][2] = {{38.0, -51.6}, {33.7, -135.4}, {36.1, 142.3}, {41.0, 47.5}, {-33.0, -51.5}, {-29.7, -135.25}, {-32.8, 142.0}, {-36.7, 47.0}};

signed int parse_rx_int() {
    int i;
    signed int ret;
    unsigned long int tens;
    unsigned char flag = 0;
    
    unsigned char temp_buffer[RX_BUFFER_SIZE];
    
    for(i = 0; rx_buffer[i] != '\0'; i++) {
        temp_buffer[i] = rx_buffer[i];
    }
    temp_buffer[i] = '\0';
    
    if((temp_buffer[0] > '9' || temp_buffer[0] < '0') && temp_buffer[0] != '-') {
        return -1;
    }
    
    if(temp_buffer[0] == '-')
        flag = 1;
    
    for(i = flag, tens = 1; temp_buffer[i] != '\0'; i++, tens *= 10);
    tens /= 10;
    
    for(i = flag, ret = 0; temp_buffer[i] != '\0'; i++, tens /= 10) {
        ret += (temp_buffer[i] - '0') * tens;
    }
    
    if(flag)
        ret = -ret;
    
    return ret;
}

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

unsigned char get_corner(float pitch_angle, float roll_angle, float offset) {
    unsigned char i, index;
    float min = 360, t = 370.0;
    
    for(i = 0, index = 0; i < 8; i++) {
        t = fabs(pitch_angle - corner_centerpoints[i][0]) + fabs(roll_angle - corner_centerpoints[i][1]);        
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

#define OFFSET_LPF 0.8

void balance_edge(float pitch_angle, float roll_angle, float loop_time, XYZ gyro, PID* motorA, PID* motorB, PID* motorC) {    
    float ks;
    float kv;  
    
    static unsigned char pre_edge = 0, pre_edge2 = 0;
    
    static bool balance = false;
    
    float angle_offset, angle, rpm;
    unsigned char port;
    
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
        
        if(edge && !pre_edge && !balance) {
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
//            motor->offset = OFFSET_LPF * motor->offset + (1-OFFSET_LPF) * (motor->offset + edge_angle_signs[edge2-1] * ks * motor->error * loop_time);
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

void balance_corner(float q[4], float loop_time, XYZ gyro_filt) {
    static PID roll, pitch, yaw;
    
    float roll_angle, pitch_angle, heading;
    float pitch_offset, roll_offset;
    static float heading_offset;        
    
    XYZ gyro_global;
    
    unsigned char corner, corner2;
    static unsigned char pre_corner = 0, pre_corner2 = 0;
    
    static bool balance = false;
    
    float A_out, B_out, C_out;
    
    float sx, sy, cx, cy, rpm_sum = 0.0;
    XYZ ra, rb, rc;
    
    float ks_a = 0.5;
    float ks_b = 0.5;
    float ks_c = 0.5;
    
    float kv = 0.1;
    
    PIDSet(&roll, 150, 0, 35); // 150 25
    PIDSet(&pitch, 150, 0, 35);
    PIDSet(&yaw, 100, 20, 5);
    roll.offset = 47.0;
    pitch.offset = -36.7;
    
    QuaternionToEuler(q, &roll_angle, &pitch_angle, &heading);
    
    corner = get_corner(pitch_angle, roll_angle, 2.0);
    corner2 = get_corner(pitch_angle, roll_angle, 20.0);
    
    if(!corner2) {    
        if(!corner2 && pre_corner2) {
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

            PIDReset(&pitch);
            PIDReset(&roll);
            PIDReset(&yaw);
        }
    } else {
        
        pitch_offset = corner_centerpoints[corner2-1][0];
        roll_offset = corner_centerpoints[corner2-1][1];
        
        if(corner && !pre_corner && !balance && corner == 8) {
            balance = true;
            USART_send_str(UART_A, "I\r");
            USART_send_str(UART_B, "I\r");
            USART_send_str(UART_C, "I\r");
            PIDReset(&pitch);
            PIDReset(&roll);
            PIDReset(&yaw);
            
            pitch.offset = pitch_offset;
            roll.offset = roll_offset;            
            
            heading_offset = heading;
        }
        
        if(balance) {

            gyro_global = MultiplyVectorQuaternion(gyro_filt, q);
            heading = LimitAngle(heading-heading_offset);

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

            rpm_sum = 0.9*rpm_sum + 0.1*(rpm_A + rpm_B + rpm_C);

            yaw.error = LimitAngle(heading - yaw.offset);
            yaw.integral += yaw.error * loop_time;
            yaw.derivative = gyro_global.z;
            PIDOutput(&yaw);

        //    yaw_angle += (gyro_global.z - gyro_spin - rpm_sum/50.0) * acc_loop_time;
        //    yaw_output = 15 * (gyro_global.z - gyro_spin - rpm_sum/50.0) + 50.0 * yaw_angle;

            roll.error = -(roll_angle - roll.offset);         
            PIDIntegrate(&roll, loop_time);
            roll.offset -= kv * roll.error * loop_time;
            roll.offset = LimitValue(roll.offset, 36, 56);

            pitch.error = pitch_angle - pitch.offset;    
            PIDIntegrate(&pitch, loop_time);
            pitch.offset += kv * pitch.error * loop_time;
            pitch.offset = LimitValue(pitch.offset, -46, -26);  

            A_out = roll.kp * ra.y*roll.error;
            A_out += roll.kd * -gyro_filt.x;
            A_out += ra.z*yaw.output;
            A_out += ks_a*rpm_A;
            A_out = LimitValue(A_out, -1000.0, 1000.0);

            B_out = pitch.kp * rb.x*-pitch.error;
            B_out += roll.kp * rb.y*-roll.error;
            B_out += pitch.kd * -gyro_filt.y;
            B_out += rb.z*yaw.output;
            B_out += ks_b*rpm_B;
            B_out = LimitValue(B_out, -1000.0, 1000.0);

            C_out = pitch.kp * rc.x*pitch.error;
            C_out += roll.kp * rc.y*-roll.error;
            C_out += pitch.kd * -gyro_filt.z;
            C_out += rc.z*yaw.output;
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
    }
    pre_corner = corner;
    pre_corner2 = corner2;
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