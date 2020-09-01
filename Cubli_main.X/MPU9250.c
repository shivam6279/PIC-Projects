#include "MPU9250.h"
#include "bitbang_I2C.h"
#include <stdbool.h>
#include <math.h>

#define ACCEL_OUT 0x3B
#define GYRO_OUT 0x43
#define TEMP_OUT 0x41
#define EXT_SENS_DATA_00 0x49
#define ACCEL_CONFIG 0x1C
#define ACCEL_FS_SEL_2G 0x00
#define ACCEL_FS_SEL_4G 0x08
#define ACCEL_FS_SEL_8G 0x10
#define ACCEL_FS_SEL_16G 0x18
#define GYRO_CONFIG 0x1B
#define GYRO_FS_SEL_250DPS 0x00
#define GYRO_FS_SEL_500DPS 0x08
#define GYRO_FS_SEL_1000DPS 0x10
#define GYRO_FS_SEL_2000DPS 0x18
#define ACCEL_CONFIG2 0x1D
#define ACCEL_DLPF_184 0x01
#define ACCEL_DLPF_92 0x02
#define ACCEL_DLPF_41 0x03
#define ACCEL_DLPF_20 0x04
#define ACCEL_DLPF_10 0x05
#define ACCEL_DLPF_5 0x06
#define CONFIG 0x1A
#define GYRO_DLPF_184 0x01
#define GYRO_DLPF_92 0x02
#define GYRO_DLPF_41 0x03
#define GYRO_DLPF_20 0x04
#define GYRO_DLPF_10 0x05
#define GYRO_DLPF_5 0x06
#define SMPDIV 0x19
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define INT_DISABLE 0x00
#define INT_PULSE_50US 0x00
#define INT_WOM_EN 0x40
#define INT_RAW_RDY_EN 0x01
#define PWR_MGMNT_1 0x6B
#define PWR_CYCLE 0x20
#define PWR_RESET 0x80
#define CLOCK_SEL_PLL 0x01
#define PWR_MGMNT_2 0x6C
#define SEN_ENABLE 0x00
#define DIS_GYRO 0x07
#define USER_CTRL 0x6A
#define I2C_MST_EN 0x20
#define I2C_MST_CLK 0x0D
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_DO 0x63
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_EN 0x80
#define I2C_READ_FLAG 0x80
#define MOT_DETECT_CTRL 0x69
#define ACCEL_INTEL_EN 0x80
#define ACCEL_INTEL_MODE 0x40
#define LP_ACCEL_ODR 0x1E
#define WOM_THR 0x1F
#define WHO_AM_I 0x75
#define FIFO_EN 0x23
#define FIFO_TEMP 0x80
#define FIFO_GYRO 0x70
#define FIFO_ACCEL 0x08
#define FIFO_MAG 0x01
#define FIFO_COUNT 0x72
#define FIFO_READ 0x74

#define AK8963_I2C_ADDR 0x0C
#define AK8963_HXL 0x03
#define AK8963_CNTL1 0x0A
#define AK8963_PWR_DOWN 0x00
#define AK8963_CNT_MEAS1 0x12
#define AK8963_CNT_MEAS2 0x16
#define AK8963_FUSE_ROM 0x0F
#define AK8963_CNTL2 0x0B
#define AK8963_RESET 0x01
#define AK8963_ASA 0x10
#define AK8963_WHO_AM_I 0x00

XYZ acc_offset, acc_gain;
XYZ gyro_offset, gyro_gain;
XYZ compass_offset, compass_gain;

#if IMU_BUFFER_SIZE > 0
XYZ acc_buffer[IMU_BUFFER_SIZE], gyro_buffer[IMU_BUFFER_SIZE], compass_buffer[IMU_BUFFER_SIZE];
#endif

void VectorReset(XYZ *v) {
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
}

XYZ VectorAdd(XYZ a, XYZ b) {
    XYZ r;
    r.x = a.x + b.x;
    r.y = a.y + b.y;
    r.z = a.z + b.z;
    return r;
}

XYZ VectorSubtract(XYZ a, XYZ b) {
    XYZ r;
    r.x = a.x - b.x;
    r.y = a.y - b.y;
    r.z = a.z - b.z;
    return r;
}

XYZ VectorScale(XYZ a, float scale) {
    XYZ r;
    r.x = a.x * scale;
    r.y = a.y * scale;
    r.z = a.z * scale;
    return r;
}

//---------------------------------------MPU6050-----------------------------------
void MPU9250Init() {
    unsigned int i;
    unsigned char temp[7];
    
    writeMPU9250Register(PWR_MGMNT_1,CLOCK_SEL_PLL);    
    writeMPU9250Register(USER_CTRL,I2C_MST_EN);
    writeMPU9250Register(I2C_MST_CTRL,I2C_MST_CLK);
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
    writeMPU9250Register(PWR_MGMNT_1,PWR_RESET);
    delay_ms(1);
    writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
    writeMPU9250Register(PWR_MGMNT_1,CLOCK_SEL_PLL);
    writeMPU9250Register(PWR_MGMNT_2,SEN_ENABLE);
    writeMPU9250Register(ACCEL_CONFIG,ACCEL_FS_SEL_2G);
    writeMPU9250Register(GYRO_CONFIG,GYRO_FS_SEL_2000DPS);
    writeMPU9250Register(ACCEL_CONFIG2,ACCEL_DLPF_20);
    writeMPU9250Register(CONFIG,GYRO_DLPF_184);
    writeMPU9250Register(SMPDIV,0x00);
    writeMPU9250Register(USER_CTRL,I2C_MST_EN);
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
    delay_ms(100);
    writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM);
    delay_ms(100);
    readAK8963Registers(AK8963_ASA, temp, 3);
    compass_gain.x = ((((float)temp[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    compass_gain.y = ((((float)temp[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    compass_gain.z = ((((float)temp[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla 
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
    delay_ms(100);
    writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);
    delay_ms(100);
    writeMPU9250Register(PWR_MGMNT_1, CLOCK_SEL_PLL);
    readAK8963Registers(AK8963_HXL, temp, 7);
        
#if IMU_BUFFER_SIZE > 0
    for(i = 0; i < IMU_BUFFER_SIZE; i++) {
        acc_buffer[i] = (XYZ){0.0, 0.0, 0.0};
        gyro_buffer[i] = (XYZ){0.0, 0.0, 0.0};
        compass_buffer[i] = (XYZ){0.0, 0.0, 0.0};
    }
#endif
    
    I2C_ReadRegisters(MPU9250_ACC_GYRO_ADDR, 119, temp, 6);
    acc_offset.x = (signed short)(temp[0] << 8 | temp[1]);
    acc_offset.y = (signed short)(temp[2] << 8 | temp[3]);
    acc_offset.z = (signed short)(temp[4] << 8 | temp[5]);

    gyro_offset.x = GYRO_X_OFFSET;
    gyro_offset.y = GYRO_Y_OFFSET;
    gyro_offset.z = GYRO_Z_OFFSET;
    
    compass_offset.x = 0.0;
    compass_offset.y = 0.0;
    compass_offset.z = 0.0;
    
    acc_gain.x = 1.0; // ACC_GRAVITY * 16.0 / 32767.f;
    acc_gain.y = acc_gain.x;
    acc_gain.z = acc_gain.x;
    
    gyro_gain.x = 2000.0 / 32767.5 * 3.14159265359f / 180.0f;
    gyro_gain.y = gyro_gain.x;
    gyro_gain.z = gyro_gain.x;
}

bool MPU9250ReadRaw(XYZ *acc, XYZ *gyro, XYZ *compass) {
    unsigned char temp[21];
    if(!I2C_ReadRegisters(MPU9250_ACC_GYRO_ADDR, ACCEL_OUT, temp, 20))
        return false;
    
    acc->x = (signed short)(temp[0] << 8 | temp[1]);
    acc->y = (signed short)(temp[2] << 8 | temp[3]);
    acc->z = (signed short)(temp[4] << 8 | temp[5]);
    
    gyro->x = (signed short)(temp[8] << 8 | temp[9]);
    gyro->y = (signed short)(temp[10] << 8 | temp[11]);
    gyro->z = (signed short)(temp[12] << 8 | temp[13]);
    
    compass->x = (signed short)(temp[15] << 8 | temp[14]);
    compass->y = (signed short)(temp[17] << 8 | temp[16]);
    compass->z = (signed short)(temp[19] << 8 | temp[18]);
}

bool MPU9250Read(XYZ *acc, XYZ *gyro, XYZ *compass) {
    MPU9250ReadRaw(acc, gyro, compass);
    
#if IMU_BUFFER_SIZE > 0
    unsigned char i;    
    for(i = (IMU_BUFFER_SIZE - 1); i >= 1; i--) {
        acc_buffer[i] = acc_buffer[i - 1];
        gyro_buffer[i] = gyro_buffer[i - 1];
        compass_buffer[i] = compass_buffer[i - 1];
    }
    acc_buffer[0] = *acc;
    gyro_buffer[0] = *gyro;
    compass_buffer[0] = *compass;
    for(i = 1; i < IMU_BUFFER_SIZE; i++) {
        *acc = VectorAdd(*acc, acc_buffer[i]);
        *gyro = VectorAdd(*gyro, gyro_buffer[i]);
        *compass = VectorAdd(*compass, compass_buffer[i]);
    }
    *acc = VectorScale(*acc, 1.0f / (float)IMU_BUFFER_SIZE);
    *gyro = VectorScale(*gyro, 1.0f / (float)IMU_BUFFER_SIZE);
    *compass = VectorScale(*compass, 1.0f / (float)IMU_BUFFER_SIZE);
#endif
    
    acc->x = (acc->x - acc_offset.x) * acc_gain.x;
    acc->y = (acc->y - acc_offset.y) * acc_gain.y;
    acc->z = (acc->z - acc_offset.z) * acc_gain.z;
    
    gyro->x = (gyro->x - gyro_offset.x) * gyro_gain.x;
    gyro->y = (gyro->y - gyro_offset.y) * gyro_gain.y;
    gyro->z = (gyro->z - gyro_offset.z) * gyro_gain.z;
    
    compass->x = (compass->x - compass_offset.x) * compass_gain.x;
    compass->y = (compass->y - compass_offset.y) * compass_gain.y;
    compass->z = (compass->z - compass_offset.z) * compass_gain.z;  
}

//bool GetRawAcc(XYZ *acc) {
//    unsigned char temp[6];
//    if(!I2C_ReadRegisters(MPU9250_ACC_GYRO_ADDR, 0x3B, temp, 6))
//        return false;
//    
//    acc->x = -(signed short)(temp[0] << 8 | temp[1]);
//    acc->y =  (signed short)(temp[2] << 8 | temp[3]);
//    acc->z = -(signed short)(temp[4] << 8 | temp[5]);
//    return true;
//}
//
//bool GetAcc(XYZ *acc) {
//    if(!GetRawAcc(acc))
//        return false;
//    
//#if IMU_BUFFER_SIZE > 0
//    unsigned char i;    
//    for(i = (IMU_BUFFER_SIZE - 1); i >= 1; i--)
//        acc_buffer[i] = acc_buffer[i - 1];
//
//    acc_buffer[0] = *acc;
//    for(i = 1; i < IMU_BUFFER_SIZE; i++)
//        *acc = VectorAdd(*acc, acc_buffer[i]);
//    
//    *acc = VectorScale(*acc, 1.0f / (float)IMU_BUFFER_SIZE);
//#endif
//
//    acc->x = (acc->x - acc_offset.x) * ACC_GRAVITY / 16384.0f * acc_gain.x;
//    acc->y = (acc->y - acc_offset.y) * ACC_GRAVITY / 16384.0f * acc_gain.y;
//    acc->z = (acc->z - acc_offset.z) * ACC_GRAVITY / 16384.0f * acc_gain.z;
//
//    return true;
//}
//
//bool GetRawGyro(XYZ *gyro) {
//    unsigned char temp[6];
//    if(!I2C_ReadRegisters(MPU9250_ACC_GYRO_ADDR, 0x43, temp, 6))
//        return false;
//
//    // Order: XH, XL, YH, YZ, ZH, ZL
//
//    gyro->x = -(signed short)(temp[0] << 8 | temp[1]);
//    gyro->y =  (signed short)(temp[2] << 8 | temp[3]);
//    gyro->z = -(signed short)(temp[4] << 8 | temp[5]);
//    return true;
//}
//
//bool GetGyro(XYZ *gyro) { 
//    if(!GetRawGyro(gyro))
//        return false;
//    
//#if IMU_BUFFER_SIZE > 0
//    unsigned char i;
//    
//    for(i = (IMU_BUFFER_SIZE - 1); i >= 1; i--)
//        gyro_buffer[i] = gyro_buffer[i - 1];
//
//    gyro_buffer[0] = *gyro;
//    for(i = 1; i < IMU_BUFFER_SIZE; i++)
//        *gyro = VectorAdd(*gyro, gyro_buffer[i]);
//
//    *gyro = VectorScale(*gyro, 1.0f / (float)IMU_BUFFER_SIZE);
//#endif
//    gyro->x = (gyro->x - gyro_offset.x) / GYRO_X_GAIN;
//    gyro->y = (gyro->y - gyro_offset.y) / GYRO_Y_GAIN;
//    gyro->z = (gyro->z - gyro_offset.z) / GYRO_Z_GAIN;
//    return true;
//}

void ComputeCompassOffsetGain(XYZ c_min, XYZ c_max) {
    compass_offset.x = (c_max.x + c_min.x) / 2.0f;
    compass_offset.y = (c_max.y + c_min.y) / 2.0f;
    compass_offset.z = (c_max.z + c_min.z) / 2.0f;

    compass_gain.x = 2.0f / (c_max.x - c_min.x);
    compass_gain.y = 2.0f / (c_max.y - c_min.y);
    compass_gain.z = 2.0f / (c_max.z - c_min.z);
}

//bool GetRawCompass(XYZ *compass) {
//    unsigned char temp[6];
//    if(!I2C_ReadRegisters(MPU9250_MAG_ADDR, 0x43, temp, 6))
//        return false;
//
//    // Order: XH, XL, YH, YZ, ZH, ZL
//
//    compass->x = -(signed short)(temp[0] << 8 | temp[1]);
//    compass->y =  (signed short)(temp[2] << 8 | temp[3]);
//    compass->z = -(signed short)(temp[4] << 8 | temp[5]);
//    return true;
//}
//
//bool GetCompass(XYZ *compass) {
//    if(!GetRawCompass(compass))
//        return false;
//    
//#if IMU_BUFFER_SIZE > 0
//    unsigned char i;
//    for(i = (IMU_BUFFER_SIZE - 1); i >= 1; i--)
//        compass_buffer[i] = compass_buffer[i - 1];
//
//    compass_buffer[0] = *compass;
//    for(i = 1; i < IMU_BUFFER_SIZE; i++)
//        *compass = VectorAdd(*compass, compass_buffer[i]);
//
//    *compass = VectorScale(*compass, 1.0f / IMU_BUFFER_SIZE);
//#endif
//
//    compass->x = (compass->x - compass_offset.x) * compass_gain.x;
//    compass->y = (compass->y - compass_offset.y) * compass_gain.y;
//    compass->z = (compass->z - compass_offset.z) * compass_gain.z;
//    
//    return true;
//}

void writeMPU9250Register(unsigned char addr, unsigned char data) {
    I2C_WriteRegisters(MPU9250_ACC_GYRO_ADDR, (unsigned char[2]){addr, data}, 2);
}

void writeAK8963Register(unsigned char addr, unsigned char data) {
    unsigned char temp;
    I2C_WriteRegisters(MPU9250_ACC_GYRO_ADDR, (unsigned char[2]){0x25, MPU9250_MAG_ADDR}, 2);
    I2C_WriteRegisters(MPU9250_ACC_GYRO_ADDR, (unsigned char[2]){0x26, addr}, 2);
    I2C_WriteRegisters(MPU9250_ACC_GYRO_ADDR, (unsigned char[2]){0x63, data}, 2);
    I2C_WriteRegisters(MPU9250_ACC_GYRO_ADDR, (unsigned char[2]){0x27, 0x81}, 2);
    readAK8963Registers(addr, &temp, 1);
}

void readAK8963Registers(unsigned char addr, unsigned char *data, unsigned char c) {
    I2C_WriteRegisters(MPU9250_ACC_GYRO_ADDR, (unsigned char[2]){0x25, MPU9250_MAG_ADDR | 0x80}, 2);
    I2C_WriteRegisters(MPU9250_ACC_GYRO_ADDR, (unsigned char[2]){0x26, addr}, 2);
    I2C_WriteRegisters(MPU9250_ACC_GYRO_ADDR, (unsigned char[2]){0x27, 0x80 | c}, 2);
    delay_ms(1);
    I2C_ReadRegisters(MPU9250_MAG_ADDR, 0x49, data, c);
}
