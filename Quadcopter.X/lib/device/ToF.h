#ifndef _ToF_H_
#define _ToF_H_

#include <stdbool.h>
#include <stdint.h>

#define LIDAR_BUFFER_LEN 7
#define LIDAR_BUFFER_LEN_MID LIDAR_BUFFER_LEN/2

#define VL6180X_ADDRESS 0x29

extern void VL53L0X_init();
extern bool VL53L0X_performSingleRefCalibration(unsigned char);
void VL53L0X_startContinuous(uint32_t);
extern uint16_t VL53L0X_readRange();
extern void VL53L0X_updateBuffer(uint16_t);
extern float VL53L0X_bufferAvg();
extern float VL53L0X_bufferDelta();

extern uint8_t VL53L0X_read8(uint8_t);
extern uint16_t VL53L0X_read16(uint8_t);
extern void VL53L0X_write8(uint8_t, uint8_t);
extern void VL53L0X_write16(uint8_t, uint16_t);

extern float lidar_buffer_data[LIDAR_BUFFER_LEN];

#endif