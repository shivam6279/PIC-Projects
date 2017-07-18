#define _XTAL_FREQ 48000000
#include <pic18f4550.h>
#include <xc.h>
#include <math.h>
#include "I2C.h"
#include "USART.h"
#include "OLED.h"
#include "10DOF.h"
#include "PWMDriver.h"

#pragma config PLLDIV = 5
#pragma config CPUDIV = OSC1_PLL2
#pragma config USBDIV = 1
#pragma config FOSC = HS
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRT = OFF
#pragma config BOR = OFF
#pragma config BORV = 2
#pragma config VREGEN = OFF
#pragma config WDT = OFF
#pragma config WDTPS = 32768
#pragma config CCP2MX = ON
#pragma config PBADEN = OFF
#pragma config LPT1OSC = OFF
#pragma config MCLRE = ON
#pragma config STVREN = OFF
#pragma config LVP = OFF
#pragma config ICPRT = OFF
#pragma config XINST = OFF
#pragma config CP0 = OFF
#pragma config CP1 = OFF
#pragma config CP2 = OFF
#pragma config CP3 = OFF
#pragma config CPB = OFF
#pragma config CPD = OFF
#pragma config WRT0 = OFF
#pragma config WRT1 = OFF
#pragma config WRT2 = OFF
#pragma config WRT3 = OFF
#pragma config WRTC = OFF
#pragma config WRTB = OFF
#pragma config WRTD = OFF
#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF
#pragma config EBTRB = OFF

unsigned char receive;
XYZ DCM_Matrix[3];

XYZ Temporary_Matrix[3];

XYZ Update_Matrix[3]; //Gyros here

XYZ Omega_Vector; //Corrected Gyro_Vector data
XYZ Omega_P;//Omega Proportional correction
XYZ Omega_I;//Omega Integrator
XYZ Omega;

void delay_ms(int x){
    int i;
    for(i = 0; i < x; i++){
        __delay_ms(1);
    }
}

void delay_us(int x){
    int i;
    for(i = 0; i < x; i++){
        __delay_us(1);
    }
}

int get_adc(unsigned char port){
    ADCON0 = port * 8 + 1;
    GO_DONE = 1;
    while(GO_DONE);
    return ((ADRESH  * 256) + ADRESL);
}

void timer_init(){
    T0CS = 0;
    T0SE = 0;
    PSA = 0;
    T0PS2 = 1;
    T0PS1 = 1;
    T0PS0 = 1;
    IPEN = 0;
    GIE = 1;
    TMR0 = 0;
    TMR0IE = 1;
    TMR0ON = 1;
}

void init(){
    TRISA = 63;
    TRISB = 0;
    TRISC = 0;
    TRISD = 1;
    TRISE = 0;
    //ADON = 1;
    //ADCON1 = 14;
    //ADCON2 = 184;
    USART_init(9600, 1);
    i2c_init();
    //SPI_init();
    receive = 0;
    
}

void interrupt ISR(){
    if(RCIF){
        receive = RCREG;
        if(OERR){
            CREN = 0;
            CREN = 1;
        }
    }
    if(TMR0IF){
        TMR0IF = 0;
    }
}

/*void LSM303D_write(unsigned char address, unsigned char data){
    i2c_start();
    i2c_send(0x3A);
    i2c_send(address);
    i2c_send(data);
    i2c_stop();
}

void init_LSM303D(){
    LSM303D_write(0x20, 0x57);
    LSM303D_write(0x21, 0);
    LSM303D_write(0x24, 0x64);
    LSM303D_write(0x25, 0x20);
    LSM303D_write(0x26, 0);
}

void LSM303D_get_acc(){
    i2c_start();
    i2c_send(0x3A);
    i2c_send(0x28 | (1 << 7));
    i2c_restart();
    i2c_send(0x3B);
    acc_x = i2c_read();
    i2c_ack();
    acc_x += i2c_read() << 8;
    i2c_ack();
    acc_y = i2c_read();
    i2c_ack();
    acc_y += i2c_read() << 8;
    i2c_ack();
    acc_z = i2c_read();
    i2c_ack();
    acc_z += i2c_read() << 8;
    i2c_nak();
    i2c_stop();
}

void LSM303D_get_compass(){
    i2c_start();
    i2c_send(0x3A);
    i2c_send(0x08 | (1 << 7));
    i2c_restart();
    i2c_send(0x3B);
    mag_x = i2c_read();
    i2c_ack();
    mag_x += i2c_read() << 8;
    i2c_ack();
    mag_y = i2c_read();
    i2c_ack();
    mag_y += i2c_read() << 8;
    i2c_ack();
    mag_z = i2c_read();
    i2c_ack();
    mag_z += i2c_read() << 8;
    i2c_nak();
    i2c_stop();
}*/

void XYZset(XYZ *v, float a, float b, float c){
    v->x = a;
    v->y = b;
    v->z = c;
}

void vector_add(XYZ *out, XYZ in1, XYZ in2){
    out->x = in1.x + in2.x;
    out->y = in1.y + in2.y;
    out->z = in1.z + in2.z;
}

void matrix_multiply(XYZ out[3], XYZ in1[3], XYZ in2[3]){
    unsigned char i;
    for(i = 0; i < 2; i++){
        out[i].x = in1[i].x * in2[0].x + in1[i].y * in2[1].x + in1[i].z * in2[2].x; 
        out[i].y = in1[i].x * in2[0].x + in1[i].y * in2[1].x + in1[i].z * in2[2].x;
        out[i].z = in1[i].x * in2[0].x + in1[i].y * in2[1].x + in1[i].z * in2[2].x;
    }
}

float vector_dot_product(XYZ v1, XYZ v2){
    float result = 0;
    result += v1.x + v2.x;
    result += v1.y + v2.y;
    result += v1.z + v2.z;
    return result; 
}

void vector_cross_product(XYZ *out, XYZ in1, XYZ in2){
    out->x = (in1.y * in2.z) - (in1.z * in2.y);
    out->y = (in1.z * in2.x) - (in1.x * in2.z);
    out->z = (in1.x * in2.y) - (in1.y * in2.x);
}

void vector_scale(XYZ *out, XYZ in, float scale){
    out->x = in.x * scale; 
    out->y = in.y * scale; 
    out->z = in.z * scale; 
}

void matrix_update(float time){
    unsigned char i;
    vector_add(&Omega, gyro, Omega_I);
    vector_add(&Omega_Vector, Omega, Omega_P);
    
    Update_Matrix[0].x = 0;
    Update_Matrix[0].y = -time * Omega_Vector.z;//-z
    Update_Matrix[0].z = time * Omega_Vector.y;//y
    Update_Matrix[1].x = time * Omega_Vector.z;//z
    Update_Matrix[1].y = 0;
    Update_Matrix[1].z = -time * Omega_Vector.x;//-x
    Update_Matrix[2].x = -time * Omega_Vector.y;//-y
    Update_Matrix[2].y = time * Omega_Vector.x;//x
    Update_Matrix[2].z = 0;
    
    matrix_multiply(Temporary_Matrix, DCM_Matrix, Update_Matrix);
    
    for(i = 0; i < 3; i++){
        DCM_Matrix[i].x += Temporary_Matrix[i].x;
        DCM_Matrix[i].y += Temporary_Matrix[i].y;
        DCM_Matrix[i].z += Temporary_Matrix[i].z;
    }
}

void normalize(void){
    float error = 0;
    XYZ temporary[3];
    float renorm = 0;

    error= -vector_dot_product(DCM_Matrix[0], DCM_Matrix[1]) * 0.5;

    vector_scale(&temporary[0], DCM_Matrix[1], error); 
    vector_scale(&temporary[1], DCM_Matrix[0], error); 

    vector_add(&temporary[0], temporary[0], DCM_Matrix[0]);
    vector_add(&temporary[1], temporary[1], DCM_Matrix[1]);

    vector_cross_product(&temporary[2], temporary[0], temporary[1]); 

    renorm = 0.5 *(3 - vector_dot_product(temporary[0], temporary[0])); 
    vector_scale(&DCM_Matrix[0], temporary[0], renorm);

    renorm = 0.5 *(3 - vector_dot_product(temporary[1], temporary[1])); 
    vector_scale(&DCM_Matrix[1], temporary[1], renorm);

    renorm = 0.5 *(3 - vector_dot_product(temporary[2], temporary[2])); 
    vector_scale(&DCM_Matrix[2], temporary[2], renorm);
}


void main(){
    init();
    XYZset(&Omega_Vector, 0, 0, 0);
    XYZset(&Omega_P, 0, 0, 0);
    XYZset(&Omega_I, 0, 0, 0);
    XYZset(&Omega, 0, 0, 0);
    delay_ms(50);
    pwm_driver_init(490);
    MPU6050_init();
    HMC5883_init();
    write_pwm(0, 2040);
    write_pwm(1, 2040);
    write_pwm(2, 2040);
    write_pwm(3, 2040);
    init_oled();
    oled_write(0);
    while(1){
        get_acc();
        set_xy(0, 0);
        write_float(acc.x, 6, 2);
        set_xy(1, 0);
        write_float(acc.y, 6, 2);
        set_xy(2, 0);
        write_float(acc.z, 6, 2);
        set_xy(3, 0);
    }
}

