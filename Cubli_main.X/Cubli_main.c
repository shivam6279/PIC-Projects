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

#define ANTI_WINDUP_MAX_BOUND 100
#define ANTI_WINDUP_MAX_ANGLE 100

#define DER_LPF 0.5

#define UART_XBee 1
#define UART_A 4
#define UART_B 2
#define UART_C 5

#define A_SETPOINT 46.8
#define B_SETPOINT -46.8
#define C_SETPOINT -45.6

#define ACC_LOOP_TIME 10000

void Corner(float[4]);

unsigned char get_face(int r, int p) { 
    if(p > -15 && p < 15) {
        if(r > -15 && r < 15){
            return 1;
        } else if(r > 165 || r < -165) {
            return 2;
        } else if(r > 75 && r < 105) {
            return 3;
        } else if(r < -75 && r > -105) {
            return 4;
        }
    } else  if(p > 75 && p < 105) {
        return 5;
    } else if(p < -75 && p > -105) {
        return 6;
    }
    return 0;
} 

void ResetQuaternion(float q[]){
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
}

char xbee_mode = 'S';
bool run_motor = true;

void main() { 
    PID motorA, motorB, motorC;
    float pre_pitch, pre_roll;
    XYZ pre_gyro;
    float acc_angle;
    float pitch_angle = 0.0, roll_angle = 0.0, heading;
    XYZ acc, gyro;
    rx XBee_rx;    
    unsigned char face;    
    float acc_loop_time;    
    bool flagA = 0, flagB = 0, flagC = 0;
    int i;
    float q[4];    
    
    float ks = 0.6;
    float ks_a = 0.7;
    float ks_b = 0.7;
    float ks_c = 0.7;
    float kv = 0.05;//0.065
    
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
    PIDSet(&motorA, 125, 0, 35);
    PIDSet(&motorB, 100, 0, 20);
    PIDSet(&motorC, 125, 0, 35);
    motorA.offset = A_SETPOINT;
    motorB.offset = B_SETPOINT;
    motorC.offset = C_SETPOINT;
    
    delay_ms(1500);
    CalibrateGyro();
    ResetQuaternion(q);    
    
    for(i = 0; i < 1000; i++) {
        StartDelayCounter();
        GetAcc(&acc);
        MadgwickQuaternionUpdateAcc(q, acc, 0.05);
        while(ms_counter() < 5);
    }
    
    T4CONbits.ON = 1;
    
    GetAcc(&acc);
    GetGyro(&gyro);
    pitch_angle = atan2(-acc.y, sqrt(pow(acc.x, 2) + pow(acc.z, 2))) * 180.0 / M_PI - roll_offset;
    
    USART_write_int(UART_A, 0);
    USART_send(UART_A, '\r');
    USART_write_int(UART_B, 0);
    USART_send(UART_B, '\r');
    USART_write_int(UART_C, 0);
    USART_send(UART_C, '\r');
    
    float p_derA = 0, p_derB = 0, p_derC = 0, pitch_acc, temp;
    
    StartDelayCounter();
    while(1) {
        if(acc_aq_counter >= ACC_LOOP_TIME) {
            acc_loop_time = (double)acc_aq_counter / 1000000.0;  
            acc_aq_counter = 0;
            
            pre_gyro = gyro;
            GetGyro(&gyro);
            GetAcc(&acc);
            
//            roll_angle = 90;
//            pitch_acc = atan2(acc.x, sqrt(pow(acc.y, 2) + pow(acc.z, 2))) * 180.0 / M_PI;
//            pitch_angle = 0.05 * pitch_acc + 0.95 * (pitch_angle - gyro.z * acc_loop_time);
            
            MadgwickQuaternionUpdateGyro(q, gyro, acc_loop_time);
            MadgwickQuaternionUpdateAcc(q, acc, acc_loop_time);
            
            pre_pitch = pitch_angle;
            pre_roll = roll_angle;
            QuaternionToEuler(q, &roll_angle, &pitch_angle, &heading);
            
            pitch_angle = 0.7*pre_pitch + 0.3*pitch_angle;
            roll_angle = 0.7*pre_roll + 0.3*roll_angle;            
            
//            gyro.x = 0.5*pre_gyro.x + 0.5*gyro.x;
//            gyro.y = 0.5*pre_gyro.y + 0.5*gyro.y;
//            gyro.z = 0.5*pre_gyro.z + 0.5*gyro.z;
            
            if(fabs(motorA.offset - roll_angle) < 1.0 && fabs(pitch_angle) < 15) {
                flagA = true;
            } else if(fabs(motorA.offset - roll_angle) > 35.0) {
                if(flagA) {
                    motorA.offset = A_SETPOINT;
                    USART_write_int(UART_A, 0);
                    USART_send(UART_A, '\r');
                    flagA = false;
                }
                motorA.integral = 0.0;
            } 
            
            if(fabs(motorB.offset - pitch_angle) < 1.0 && fabs(roll_angle) < 15) {
                flagB = true;
            } else if(fabs(motorB.offset - pitch_angle) > 35.0) {
                if(flagB) {
                    motorB.offset = B_SETPOINT;
                    USART_write_int(UART_B, 0);
                    USART_send(UART_B, '\r');
                    flagB = false;
                }
                motorB.integral = 0.0;
            } 
            
            if(fabs(motorC.offset - pitch_angle) < 1.0 && fabs(90 - roll_angle) < 15) {
                flagC = true;
            } else if(fabs(motorC.offset - pitch_angle) > 35.0) {
                if(flagC) {
                    motorC.offset = C_SETPOINT;
                    USART_write_int(UART_C, 0);
                    USART_send(UART_C, '\r');
                    flagC = false;
                }
                motorC.integral = 0.0;
            } 
            
            if(fabs(-36.0 - pitch_angle) < 5 && fabs(46.0 - roll_angle) < 5) {
                Corner(q);
            }
            
            if(flagA) {
                motorA.p_error = motorA.error;
                motorA.error = -(roll_angle - motorA.offset);
                
//                p_derA = motorA.derivative;
//                motorA.derivative = (1.0-DER_LPF) * ((motorA.error - motorA.p_error) / acc_loop_time) + DER_LPF * p_derA;
                motorA.derivative = -gyro.x;              
                
//                if(motorA.p_error * motorA.error < 0) {
//                    motorA.integral = 0.0;
//                }
                PIDIntegrate(&motorA, acc_loop_time);

                motorA.offset += ks_a * motorA.error * acc_loop_time;
                motorA.offset = LimitValue(motorA.offset, 35, 55);   
            }
            
            if(flagB) {
                motorB.p_error = motorB.error;
                motorB.error = -(pitch_angle - motorB.offset);
                
//                p_derB = motorB.derivative;
//                motorB.derivative = (1.0-DER_LPF) * ((motorB.error - motorB.p_error) / acc_loop_time) + DER_LPF * p_derB;
                motorB.derivative = -gyro.y;         
                
//                if(motorB.p_error * motorB.error < 0) {
//                    motorB.integral = 0.0;
//                }
                PIDIntegrate(&motorB, acc_loop_time);

                motorB.offset += ks_b * motorB.error * acc_loop_time;
                motorB.offset = LimitValue(motorB.offset, -55, -35);   
            }
            
            if(flagC) {
                motorC.p_error = motorC.error;
                motorC.error = (pitch_angle - motorC.offset);   
                
//                p_derC = motorC.derivative;
//                motorC.derivative = (1.0-DER_LPF) * ((motorC.error - motorC.p_error) / acc_loop_time) + DER_LPF * p_derC;
                motorC.derivative = -gyro.z;                
                
//                if(motorC.p_error * motorC.error < 0) {
//                    motorC.integral = 0.0;
//                }
                PIDIntegrate(&motorC, acc_loop_time);

                motorC.offset -= ks_c * motorC.error * acc_loop_time;
                motorC.offset = LimitValue(motorC.offset, -56, -36);   
            }
            
            if(mode == 'R') {
                run_motor = true;
            } else if(mode == 'O') {
                run_motor = false;
            } else {
                xbee_mode = mode;
            }
            
            if(flagA) {
                PIDOutput(&motorA);
                motorA.output += kv*rpm_A;
                motorA.output =  LimitValue(motorA.output, -800.0, 800.0);  
                
                if(run_motor) {
                    USART_write_int(UART_A, motorA.output);
                    USART_send(UART_A, '\r');                
                }                
            }
            
            if(flagB) {
                PIDOutput(&motorB);
                motorB.output += kv*rpm_B;
                motorB.output = LimitValue(motorB.output, -800.0, 800.0);
                
                if(run_motor) {
                    USART_write_int(UART_B, motorB.output);
                    USART_send(UART_B, '\r');                
                }
            }
            
            if(flagC) {
                PIDOutput(&motorC);
                motorC.output += 0.083*rpm_C;
                motorC.output = LimitValue(motorC.output, -800.0, 800.0); 
                
                if(run_motor) {
                    USART_write_int(UART_C, motorC.output);
                    USART_send(UART_C, '\r');  
                }
            }
        }
           
        if(ms_counter() > 25) {
            StartDelayCounter();
            if(xbee_mode == 'A') {
                USART_write_float(1, pitch_angle, 4);     
                USART_send_str(UART_XBee, ", ");
                USART_write_float(1, roll_angle, 4);
                USART_send_str(UART_XBee, ", ");
                USART_write_float(1, heading, 4);
                USART_send_str(UART_XBee, "\n");
                
            } else if(xbee_mode == 'G') {
                USART_write_float(1, gyro.x, 4);     
                USART_send_str(UART_XBee, ", ");
                USART_write_float(1, gyro.y, 4);
                USART_send_str(UART_XBee, ", ");
                USART_write_float(1, gyro.z, 4);
                USART_send_str(UART_XBee, "\n");
                
            } else if(xbee_mode == 'D') {
                if(flagA) {
                    USART_write_float(1, motorA.derivative, 4);     
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, -gyro.x, 4);
                    USART_send_str(UART_XBee, "\n");
                }else if(flagB) {
                    USART_write_float(1, motorB.derivative, 4);     
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, -gyro.y, 4);
                    USART_send_str(UART_XBee, "\n");
                }else if(flagC) {
                    USART_write_float(1, motorC.derivative, 4);     
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, -gyro.z, 4);
                    USART_send_str(UART_XBee, "\n");
                }
                   
            } else if(xbee_mode == 'V') {
                USART_write_float(1, rpm_A, 4);     
                USART_send_str(UART_XBee, ", ");
                USART_write_float(1, rpm_B, 4);
                USART_send_str(UART_XBee, ", ");
                USART_write_float(1, rpm_C, 4);
                USART_send_str(UART_XBee, "\n");
                
            }  else if(xbee_mode == 'P') {
                if(flagA) {
                    USART_write_float(1, motorA.error*motorA.kp, 4);    
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, motorA.integral*motorA.ki, 4);
                    USART_send_str(UART_XBee, ", ");                    
                    USART_write_float(1, motorA.derivative*motorA.kd, 4);
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, motorA.output, 4);
                    USART_send_str(UART_XBee, "\n");
                } else if(flagB) {
                    USART_write_float(1, motorB.error*motorB.kp, 4);    
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, motorB.integral*motorB.ki, 4);
                    USART_send_str(UART_XBee, ", ");                    
                    USART_write_float(1, motorB.derivative*motorB.kd, 4);
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, motorB.output, 4);
                    USART_send_str(UART_XBee, "\n");
                } else if(flagC) {
                    USART_write_float(1, motorC.error*motorC.kp, 4);    
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, motorC.integral*motorC.ki, 4);
                    USART_send_str(UART_XBee, ", ");                    
                    USART_write_float(1, motorC.derivative*motorC.kd, 4);
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, motorC.output, 4);
                    USART_send_str(UART_XBee, "\n");
                } else {
                    USART_write_float(1, pitch_angle, 4);     
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, roll_angle, 4);
                    USART_send_str(UART_XBee, "\n");
                }
                
            } else {
                if(flagA) {
                    USART_write_float(1, roll_angle, 4);     
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, motorA.offset, 4);
                    USART_send_str(UART_XBee, "\n");
                } else if(flagB) {
                    USART_write_float(1, pitch_angle, 4);     
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, motorB.offset, 4);
                    USART_send_str(UART_XBee, "\n");
                } else if(flagC) {
                    USART_write_float(1, pitch_angle, 4);     
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, motorC.offset, 4);
                    USART_send_str(UART_XBee, "\n");
                } else {
                    USART_write_float(1, pitch_angle, 4);     
                    USART_send_str(UART_XBee, ", ");
                    USART_write_float(1, roll_angle, 4);
                    USART_send_str(UART_XBee, "\n");
                }
            }            
        }
    }
}

#define c30 0.86602540378443864676372317075294
#define s30 0.5

void Corner(float q[4]) {
    PID roll, pitch;
    float pitch_angle = 0.0, roll_angle = 0.0, heading;
    XYZ acc, gyro; 
    float acc_loop_time;  
    float gyro_spin = 0, gyro_mag, gyro_mag_sp;
    
    float A_out, B_out, C_out;
    
    float ks_a = 0.05; //0.05
    float ks_b = 0.05; //0.05
    float ks_c = 0.05; //0.074
    
    PIDSet(&roll, 100, 0, 25);
    PIDSet(&pitch, 100, 0, 25);
    roll.offset = 46.0;
    pitch.offset = -36.0;
    
    esc_counter = 0;
    
    while(1) {
//        if(esc_counter > 15000000 && gyro_spin == 0) {
//            gyro_mag_sp = 5.0;
//            esc_counter = 0;
//        } else if(esc_counter > 10000000 && gyro_spin > 5) {
//            gyro_mag_sp = -5.0;
//        }
//        
        
        if(acc_aq_counter >= ACC_LOOP_TIME) {
            acc_loop_time = (double)acc_aq_counter / 1000000.0;  
            acc_aq_counter = 0;

            GetGyro(&gyro);
            GetAcc(&acc);
            
//            if(gyro_mag_sp != 0) {
//                gyro_mag = sqrt(gyro.x*gyro.x + gyro.y*gyro.y + gyro.z*gyro.z) - fabs(gyro_mag_sp);
//                gyro_spin = gyro_mag_sp < 0 ? -gyro_mag: gyro_mag; 
//            }
            
            MadgwickQuaternionUpdateGyro(q, gyro, acc_loop_time);
            MadgwickQuaternionUpdateAcc(q, acc, acc_loop_time);
            QuaternionToEuler(q, &roll_angle, &pitch_angle, &heading);
            
            if(get_face(roll_angle, pitch_angle) != 0)
                return;
            
            roll.error = roll_angle - roll.offset;         
            PIDIntegrate(&roll, acc_loop_time);
            roll.offset -= 0.6 * roll.error * acc_loop_time;
            roll.offset = LimitValue(roll.offset, 36, 56);
            
            pitch.error = pitch_angle - pitch.offset;    
            PIDIntegrate(&pitch, acc_loop_time);
            pitch.offset -= 0.6 * pitch.error * acc_loop_time;
            pitch.offset = LimitValue(pitch.offset, -46, -26);
            
            if(mode == 'R') {
                run_motor = true;
            } else if(mode == 'O') {
                run_motor = false;
            } else {
                xbee_mode = mode;
            }
            
            A_out = 125.0 * -roll.error + 15.0 * -(gyro.x-gyro_spin) + ks_a*rpm_A;
            A_out = LimitValue(A_out, -800.0, 800.0);
            
            if(run_motor) {
                USART_write_int(UART_A, A_out);
                USART_send(UART_A, '\r');                
            }     

            B_out = c30 * 125.0 * -pitch.error + s30 * 55.0 * roll.error + 15.0 * -(gyro.y-gyro_spin) + ks_b*rpm_B;
            B_out = LimitValue(B_out, -800.0, 800.0);

            if(run_motor) {
                USART_write_int(UART_B, B_out);
                USART_send(UART_B, '\r');                
            }
            //c30 * kp * pitch.error
            //-s30 * kp * roll.error

            C_out = c30 * 125.0 * pitch.error + s30 * 55.0 * roll.error + 20.0 * -(gyro.z-gyro_spin) + ks_c*rpm_C;
            C_out = LimitValue(C_out, -800.0, 800.0);

            if(run_motor) {
                USART_write_int(UART_C, C_out);
                USART_send(UART_C, '\r');  
            }
        }
           
        if(ms_counter() > 25) {
            StartDelayCounter();
            USART_write_float(1, pitch_angle, 4);     
            USART_send_str(UART_XBee, ", ");
            USART_write_float(1, pitch.offset, 4);     
            USART_send_str(UART_XBee, ", ");
            USART_write_float(1, roll_angle, 4);
            USART_send_str(UART_XBee, ", ");
            USART_write_float(1, roll.offset, 4);
            USART_send_str(UART_XBee, "\n");      
        }
    }
}
