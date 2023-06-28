#ifndef _10DOF_H_
#define _10DOF_H_

#define MAX_BUFFER_SIZE 10

#define gyro_sensitivity 98

#define gyro_y_offset -180.5
#define gyro_x_offset -138.5
#define gyro_z_offset -90.25

#define compass_x_offset 420
#define compass_y_offset -315
#define compass_z_offset -95

#define compass_x_gain 1.351351351
#define compass_y_gain -1.587301587
#define compass_z_gain 1.538461538

typedef struct{
    float x,y,z;
} XYZ;

extern void MPU6050_write(unsigned char address, unsigned char data);
extern void MPU6050_init();
extern void HMC5883_init();
extern void calibrate_gyros();
extern void get_acc();
extern void get_gyro();
extern void get_compass();
extern float get_acc_x_angle();
extern float get_acc_y_angle();
extern float get_compensated_heading(float sinx, float cosx, float siny, float cosy);

extern XYZ acc, acc_buffer[MAX_BUFFER_SIZE];
extern XYZ gyro, gyro_buffer[MAX_BUFFER_SIZE];
extern XYZ compass, compass_buffer[MAX_BUFFER_SIZE];

extern unsigned char buffer_size;

#endif
