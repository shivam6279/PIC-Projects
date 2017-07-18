#ifndef _10DOF_H_
#define _10DOF_H_

#ifdef micro
#define MAX_BUFFER_SIZE 10

#define gyro_x_offset 100
#define gyro_y_offset -245
#define gyro_z_offset -15

#define gyro_x_gain 98
#define gyro_y_gain -98
#define gyro_z_gain 106

#define compass_x_min -920.0f
#define compass_x_max -230.0f

#define compass_y_min -130.0f
#define compass_y_max 550.0f

#define compass_z_min -600.0f
#define compass_z_max 0.0f

#define compass_x_offset (compass_x_max + compass_x_min) / 2.0f
#define compass_y_offset (compass_y_max + compass_y_min) / 2.0f 
#define compass_z_offset (compass_z_max + compass_z_min) / 2.0f 
#define compass_x_gain (500.0f / ((compass_x_max - compass_x_min) / 2.0f))
#define compass_y_gain -(500.0f / ((compass_y_max - compass_y_min) / 2.0f))
#define compass_z_gain (500.0f / ((compass_z_max - compass_z_min) / 2.0f))

#define oversampling 3
#define SeaLevelPressure 101325UL
#endif

#ifdef mini
#define MAX_BUFFER_SIZE 10

#define gyro_x_offset -32
#define gyro_y_offset -310
#define gyro_z_offset -45

#define gyro_x_gain 98
#define gyro_y_gain -98
#define gyro_z_gain 106

#define compass_x_min -340.0f
#define compass_x_max 600.0f

#define compass_y_min -320.0f
#define compass_y_max 439.0f

#define compass_z_min -360.0f
#define compass_z_max 180.0f

#define compass_x_offset (compass_x_max + compass_x_min) / 2
#define compass_y_offset (compass_y_max + compass_y_min) / 2
#define compass_z_offset (compass_z_max + compass_z_min) / 2
#define compass_x_gain (500.0f / (compass_x_max - compass_x_min) / 2)
#define compass_y_gain -(500.0f / (compass_y_max - compass_y_min) / 2)
#define compass_z_gain (500.0f / (compass_z_max - compass_z_min) / 2)

#define oversampling 3
#define SeaLevelPressure 101325UL
#endif

#ifdef big
#define MAX_BUFFER_SIZE 10

#define gyro_x_offset 105
#define gyro_y_offset -220
#define gyro_z_offset -7

#define gyro_x_gain 98
#define gyro_y_gain -98
#define gyro_z_gain 106

#define compass_x_min -190.0f
#define compass_x_max 650.0f

#define compass_y_min -240.0f
#define compass_y_max 405.0f

#define compass_z_min -650.0f
#define compass_z_max -80.0f

#define compass_x_offset (compass_x_max + compass_x_min) / 2
#define compass_y_offset (compass_y_max + compass_y_min) / 2
#define compass_z_offset (compass_z_max + compass_z_min) / 2
#define compass_x_gain (500.0f / (compass_x_max - compass_x_min) / 2)
#define compass_y_gain -(500.0f / (compass_y_max - compass_y_min) / 2)
#define compass_z_gain (500.0f / (compass_z_max - compass_z_min) / 2)

#define oversampling 3
#define SeaLevelPressure 101325UL
#endif

typedef struct{
    float x,y,z;
} XYZ;

//MPU6050
void MPU6050_init();
void get_acc();
void get_gyro();
float get_acc_x_angle();
float get_acc_y_angle();
//HMC5883
void HMC5883_init();
void get_compass();
float get_compensated_heading(float sinx, float cosx, float siny, float cosy);
//BMP180
void BMP180_init();
void get_raw_temperature();
float get_altitude();

XYZ acc, acc_buffer[MAX_BUFFER_SIZE];
XYZ gyro, gyro_buffer[MAX_BUFFER_SIZE];
XYZ compass, compass_buffer[MAX_BUFFER_SIZE];

signed short ac1, ac2, ac3, b1, b2, mb, mc, md;
unsigned short ac4, ac5, ac6;
unsigned long int UT;
float B5;

unsigned char buffer_size;

//-----------------------------------------MPU6050---------------------------------
void MPU6050_init(){
    unsigned char i;
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6B, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x19, 0x07}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x1A, 0x03}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x1B, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x1C, 0x00}, 2);
    for(i = 0x1D; i <= 0x23; i++){
        i2c5_write_registers(0xD0, (unsigned char[2]){i, 0x00}, 2);
    }
    i2c5_write_registers(0xD0, (unsigned char[2]){0x24, 0x40}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x25, 0x8C}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x26, 0x02}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x27, 0x88}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x28, 0x0C}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x29, 0x0A}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x2A, 0x81}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x64, 0x01}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x67, 0x03}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x01, 0x80}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x34, 0x04}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x64, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x64, 0x01}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x20}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x34, 0x13}, 2);
    for(i = 0; i < 5; i++){
        acc_buffer[i].x = 0;
        acc_buffer[i].y = 0;
        acc_buffer[i].z = 0;
        gyro_buffer[i].x = 0;
        gyro_buffer[i].y = 0;
        gyro_buffer[i].z = 0;
    }
}

void get_acc(){
    unsigned char temp[6];
    unsigned char i;
    if(buffer_size > 0){
        for(i = (buffer_size - 1); i >= 1; i--){
            acc_buffer[i].x = acc_buffer[i - 1].x;
            acc_buffer[i].y = acc_buffer[i - 1].y;
            acc_buffer[i].z = acc_buffer[i - 1].z;
        }
    }
    i2c5_read_registers(0xD0, 0x3B, temp, 6);
    acc.y = (signed short)(temp[0] << 8 | temp[1]);
    acc.x = (signed short)(temp[2] << 8 | temp[3]) * (-1);
    acc.z = (signed short)(temp[4] << 8 | temp[5]) * (-1);
    if(buffer_size > 0){
        acc_buffer[0].x = acc.x;
        acc_buffer[0].y = acc.y;
        acc_buffer[0].z = acc.z;
        for(i = 1; i < buffer_size; i++){
            acc.x += acc_buffer[i].x;
            acc.y += acc_buffer[i].y;
            acc.z += acc_buffer[i].z;
        }
        acc.x /= buffer_size;
        acc.y /= buffer_size;
        acc.z /= buffer_size;
    }
}

void get_gyro(){
    unsigned char temp[6];
    unsigned char i;
    if(buffer_size > 0){
        for(i = (buffer_size - 1); i >= 1; i--){
            gyro_buffer[i].x = gyro_buffer[i - 1].x;
            gyro_buffer[i].y = gyro_buffer[i - 1].y;
            gyro_buffer[i].z = gyro_buffer[i - 1].z;
        }
    }
    i2c5_read_registers(0xD0, 0x43, temp, 6);
    gyro.y = (signed short)(temp[0] << 8 | temp[1]);
    gyro.x = (signed short)(temp[2] << 8 | temp[3]);
    gyro.z = (signed short)(temp[4] << 8 | temp[5]);
    if(buffer_size > 0){
        gyro_buffer[0].x = gyro.x;
        gyro_buffer[0].y = gyro.y;
        gyro_buffer[0].z = gyro.z;
        for(i = 1; i < buffer_size; i++){
            gyro.x += gyro_buffer[i].x;
            gyro.y += gyro_buffer[i].y;
            gyro.z += gyro_buffer[i].z;
        }
        gyro.x /= buffer_size;
        gyro.y /= buffer_size;
        gyro.z /= buffer_size;
    }
    gyro.x = (gyro.x - gyro_x_offset) / gyro_x_gain;
    gyro.y = (gyro.y - gyro_y_offset) / gyro_y_gain;
    gyro.z = (gyro.z - gyro_z_offset) / gyro_z_gain;
}

float get_acc_x_angle(){
    return (atan2(acc.y, sqrt(acc.z * acc.z + acc.x * acc.x)));
}

float get_acc_y_angle(){
    return (atan2(acc.x, sqrt(acc.z * acc.z + acc.y * acc.y)));
}

//-----------------------------------------HMC5883---------------------------------

void HMC5883_init(){
    unsigned char i;
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x37, 0x02}, 2);
    for(i = 0; i < 5; i++){
        compass_buffer[i].x = 0;
        compass_buffer[i].y = 0;
        compass_buffer[i].z = 0;
    }
    i2c5_write_registers(0x3C, (unsigned char[2]){0, 0x14}, 2);
    i2c5_write_registers(0x3C, (unsigned char[2]){1, 0x20}, 2);
    i2c5_write_registers(0x3C, (unsigned char[2]){2, 0x00}, 2);
    
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x20}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x37, 0x00}, 2);
    for(i = 0; i < 5; i++){
        compass_buffer[i].x = 0;
        compass_buffer[i].y = 0;
        compass_buffer[i].z = 0;
    }
}

void get_compass(){
    unsigned char temp[6];
    unsigned char i;
    if(buffer_size > 0){
        for(i = (buffer_size - 1); i >= 1; i--){
            compass_buffer[i].x = compass_buffer[i - 1].x;
            compass_buffer[i].y = compass_buffer[i - 1].y;
            compass_buffer[i].z = compass_buffer[i - 1].z;
        }
    }
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x00}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x37, 0x02}, 2);
    i2c5_read_registers(0x3C, 0x03, temp, 6);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x6A, 0x20}, 2);
    i2c5_write_registers(0xD0, (unsigned char[2]){0x37, 0x00}, 2);
    compass.y = (signed short)(temp[0] << 8 | temp[1]);//y
    compass.z = (signed short)(temp[2] << 8 | temp[3]);
    compass.x = (signed short)(temp[4] << 8 | temp[5]);//x
    if(buffer_size > 0){
        compass_buffer[0].x = compass.x;
        compass_buffer[0].y = compass.y;
        compass_buffer[0].z = compass.z;
        for(i = 1; i < buffer_size; i++){
            compass.x += compass_buffer[i].x;
            compass.y += compass_buffer[i].y;
            compass.z += compass_buffer[i].z;
        }
        compass.x /= buffer_size;
        compass.y /= buffer_size;
        compass.z /= buffer_size;
    }
    compass.x = (compass.x - compass_x_offset) * compass_x_gain;
    compass.y = (compass.y - compass_y_offset) * compass_y_gain;
    compass.z = (compass.z - compass_z_offset) * compass_z_gain;
}

float get_compensated_heading(float sinx, float cosx, float siny, float cosy){
    return atan2((compass.z * siny - compass.y * cosy), (compass.x * cosx + compass.y * sinx * siny + compass.z * cosy * sinx));
}

//-----------------------------------------BMP180---------------------------------

void BMP180_init(){
    unsigned char temp[22];
    i2c5_read_registers(0xEE, 0xAA, temp, 22);
    ac1 = (signed short)(temp[0] << 8 | temp[1]);
    ac2 = (signed short)(temp[2] << 8 | temp[3]);
    ac3 = (signed short)(temp[4] << 8 | temp[5]);
    ac4 = temp[6] << 8 | temp[7];
    ac5 = temp[8] << 8 | temp[9];
    ac6 = temp[10] << 8 | temp[11];
    b1 = (signed short)(temp[12] << 8 | temp[13]);
    b2 = (signed short)(temp[14] << 8 | temp[15]);
    mb = (signed short)(temp[16] << 8 | temp[17]);
    mc = (signed short)(temp[18] << 8 | temp[19]);
    md = (signed short)(temp[20] << 8 | temp[21]);
}

void get_raw_temperature(){
    float X1, X2;
    unsigned char t[2];
    i2c5_read_registers(0xEE, 0xF6, t, 2);
    UT = t[0] << 8 | t[1];
    X1 = ((float)UT - (float)ac6) * ((float)ac5 / 32768.0f);
    X2 = ((float)(mc * 2048.0f)) / (X1 + (float)md);
    B5 = X1 + X2;
}

float get_temperature() {
    float temp;
    temp = (B5 + 8) / 16.0f;
    temp /= 10;
    return temp;
}

float get_altitude(){
    long int UP, p;
    float altitude, X1, X2, X3, B3, B6;
    unsigned long int B4, B7;
    unsigned char t[3];
    
    i2c5_read_registers(0xEE, 0xF6, t, 3);
    UP = ((t[0] << 16) | (t[1] << 8) | (t[2]));
    UP >>= (8 - oversampling);

    // do pressure calcs
    B6 = B5 - 4000;
    X1 = ((float)b2 * ((B6 * B6) / 4096.0f)) / 2048.0f;
    X2 = ((float)ac2 * B6) / 2048.0f;
    X3 = X1 + X2;
    B3 = (((float)ac1 * 4 + X3) * pow(2, oversampling) + 2) / 4;

    X1 = ((float)ac3 * B6) / 8192.0f;
    X2 = ((float)b1 * ((B6 * B6) / 4096.0f)) / 65536.0f;
    X3 = ((X1 + X2) + 2)/ 4.0f;
    B4 = (float)ac4 * ((X3 + 32768)) / 32768.0f;
    B7 = ((float)UP - B3) * (50000 / pow(2, oversampling));

    if(B7 < 0x80000000) p = (B7 * 2) / B4;
    else p = (B7 / B4) * 2;
    X1 = ((float)p / 256.0f) * ((float)p / 256.0f);
    X1 = (X1 * 3038) / 65536.0f;
    X2 = (-7357 * (float)p) / 65536.0f;

    p = p + ((X1 + X2 + 3791) / 16.0f);
    altitude = 44330 * (1.0 - pow((float)p / SeaLevelPressure, 0.1903));
    return altitude;
}

#endif
