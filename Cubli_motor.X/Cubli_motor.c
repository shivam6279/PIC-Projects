#include "pragma.h"
#include <xc.h>
#include <sys/attribs.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "pic32.h"
#include "PWM.h"
#include "BLDC.h"
#include "USART.h"
#include "MPU6050.h"
#include "bitbang_I2C.h"
#include "tones.h"
#include "EEPROM.h"
#include "SPI.h"
#include "AHRS.h"

/* 
 * UART CODES:
 * 
 * I: LED On
 * O : LED Off
 * 
 * P: Power mode
 * R: RPM Mode (PID control RPM)
 * A: Angle mode PID control angle)
 * 
 * T: Play tone
 */

#define FOC_TIMER_ON T4CONbits.ON

#define SETPOINT_CENTER 40.0f

#define AUTO_STOP 1
#define ESC 1

unsigned char board_id = 0;

void ResetQuaternion(float q[]){
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
}

signed int parse_rx() {
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

void main() {
    int i;
    XYZ acc, pre_gyro, gyro;    
    float roll_offset;
    float setpoint_center;
    signed int a = 0;
    int out_int = 0;
    float roll_angle, pitch_angle, heading;
    unsigned char mode_temp;
    float q[4];
    
    PICInit();
    
    QEI_init();
    mode = MODE_POWER;
    
    USART3_init(250000);    
    timer2_init(1000);      // ms delay
    timer3_init(4500);      // us delay
    timer4_init(10000);     // FOC
    timer5_init(50);        // speaker  
    timer6_init(1000);      // velocity
    
    EEPROM_init();
            
    PwmInit(24000);    
    delay_ms(200);    
    MPU6050Init();
    
    board_id = EEPROM_read(ID_ADDR);    
//    LED = 1;
//    MetroidSaveTheme(board_id);
//    LED = 0;
 
#if ESC == 0
    ResetQuaternion(q);
    for(i = 0; i < 2000; i++) {
        StartDelaymsCounter();
        GetAcc(&acc);
        GetGyro(&gyro);
        MadgwickQuaternionUpdateAcc(q, acc, 0.05);
        MadgwickQuaternionUpdateGyro(q, gyro, 0.001);
        while(ms_counter() < 1);
    }
    QuaternionToEuler(q, &roll_angle, &pitch_angle, &heading);
#endif  
    
//    mode = MODE_OFF;    
//    while(1) {
//        MotorPhase(1, 0.075);
//        delay_ms(500);
//        MotorPhase(2, 0.075);
//        delay_ms(500);
//        MotorPhase(3, 0.075);
//        delay_ms(500);
//        MotorPhase(4, 0.075);
//        delay_ms(500);
//        MotorPhase(5, 0.075);
//        delay_ms(500);
//        MotorPhase(6, 0.075);
//        delay_ms(500);
//    }
    
//    setPhaseVoltage(0.075, 0);    
//    FOC_TIMER_ON = 1; 
//    mode = MODE_OFF;
//    while(1) {
//        USART3_write_float(GetPosition(), 2);
//        USART3_send('\n');
//        delay_ms(150);
//    }
    
//    Write_Motor_Offset(15);
//    Write_Roll_Offset(1.2);
//    Write_Roll_Setpoint(42.2);
//    CalibrateGyro();
    
    motor_zero_angle = Read_Motor_Offset();
    setpoint_center = Read_Roll_Setpoint();
    roll_offset = Read_Roll_Offset();
    gyro_offset = ReadGyroCalibration();
    
    ResetMotorPID();
    FOC_TIMER_ON = 1;
    MotorOff();
     
    bool flag = false;
    
    float setpoint = setpoint_center, pre_set;    
    float pre_roll = 0.0, roll = 0.0, roll_acc, sum = 0.0;
    float pre_der = 0.0, der = 0.0, out = 0.0, err = 0.0, ttt; 
    
    GetAcc(&acc);
    GetGyro(&gyro);
    roll = atan2(acc.x, sqrt(pow(acc.y, 2) + pow(acc.z, 2))) * 180.0 / M_PI - roll_offset;    
    
    const float kp = 150.0;
    const float ki = 10.0;
    const float kd = 35.0;
    const float ks = 0.83;
    const float kt = 0.3;
    
    StartDelaymsCounter();
    while(1) {        
        if(rx_rdy) {            
            reset_ms_counter2();           
            a = parse_rx();
            rx_rdy = 0;

            if(mode == MODE_POWER) {
                SetPower((float)a / 2000.0);
            } else if(mode == MODE_RPM) {
                SetRPM(a);
            } else if(mode == MODE_POS) {
                SetPosition(a);
            }
        }
        
//        if(play_tone) {
//            if(mode == MODE_POWER) {
//                SetPower(0);
//            } else if(mode == MODE_RPM) {
//                SetRPM(0);
//            } else if(mode == MODE_POS) {
//                SetPosition(0);
//            }
//            mode_temp = mode;
//            mode = MODE_OFF;
//            LED = 1;
//            MetroidSaveTheme(board_id);
//            LED = 0;
//            mode = mode_temp;
//            play_tone = 0;
//        }
        
    #if AUTO_STOP == 1
        if(ms_counter2() > 100) {
            SetPower(0);
        }
    #endif       
        
    #if ESC == 0
        if(us_counter() > 50) {            
            StopDelayusCounter();
            reset_us_counter();
            rx_rdy = 1;
        }
    #endif
        if(ms_counter() >= 2) {            
            float deltat = (float)ms_counter() / 1000.0;
            reset_ms_counter();
            
        #if ESC == 0
            
            pre_gyro = gyro;
            GetAcc(&acc);
            GetGyro(&gyro);   
            
//            MadgwickQuaternionUpdateGyro(q, gyro, deltat);
//            MadgwickQuaternionUpdateAcc(q, acc, deltat);
//            QuaternionToEuler(q, &roll_angle, &pitch_angle, &heading);
            
            roll_acc = atan2(acc.x, sqrt(pow(acc.y, 2) + pow(acc.z, 2))) * 180.0 / M_PI - roll_offset;
            roll = 0.05 * roll_acc + 0.95 * (roll - gyro.z * deltat);
//            roll = pitch_angle;
            
//            der = 0.2 * (-gyro.z) + 0.8 * der;
            der = -gyro.z;   
            
            if(fabs(setpoint - roll) < 1.0) {
                flag = true;
                LED = 1;
            } else if(fabs(45.0 - roll) > 25.0) {
                if(flag) {
                    setpoint = setpoint_center;
                    SetPower(0.0);
                    SetRPM(0.0);
                    flag = false;
                }
                LED = 0;
                sum = 0.0;
                out = 0;
            }
            
            if(flag) {
                err = roll - setpoint;
                sum += err * deltat;
                
                pre_set = setpoint;
                setpoint += kt*err*deltat;
//                setpoint = 0.05*(setpoint - kt*err*deltat) + 0.95*pre_set;
                setpoint = setpoint > 60 ? 60: setpoint < 30 ? 30: setpoint;
                
                out = kp*err + ki*sum + kd*der + ks*GetRPM();
                out = out >= 1000.0 ? 1000.0: out <= -1000.0 ? -1000.0: out;
                
                out_int = out;
                i = 0;
                if(out_int < 0)
                    rx_buffer[i++] = '-';
                rx_buffer[i++] = (abs(out_int) / 1000) % 10 + '0';
                rx_buffer[i++] = (abs(out_int) / 100) % 10 + '0';
                rx_buffer[i++] = (abs(out_int) / 10) % 10 + '0';
                rx_buffer[i++] = abs(out_int)  % 10 + '0';
                rx_buffer[i] = '\0';
                rx_rdy = 1;
                StartDelayusCounter();
                
//                SetPower((int)out / 2000.0);
            }
            
            USART3_write_float(roll, 2);
            USART3_send_str(", ");
            USART3_write_float(der, 2);
            USART3_send('\n');
        #else
            
            USART3_write_float(GetRPM(), 2);
//            USART3_send_str(", ");
//            USART3_write_float(GetRPM_der(), 2);
            USART3_send('\n');
    #endif
        }
    }
}