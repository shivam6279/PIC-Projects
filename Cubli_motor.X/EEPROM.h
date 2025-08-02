#ifndef _EEPROM_H
#define _EEPROM_H

#include <inttypes.h>

extern uint8_t eeprom_board_id;
extern float eeprom_zero_offset;
extern uint8_t eeprom_polepairs;
extern float eeprom_encoder_calib_data[32];
extern float eeprom_pid_angle[3];
extern float eeprom_pid_rpm[3];
extern float eeprom_pid_foc_iq[3];
extern float eeprom_pid_foc_id[3];

extern unsigned char board_id;

extern void EEPROM_init();

extern uint32_t EEPROM_read();
extern float EEPROM_readFloat();

extern void EEPROM_write(uint32_t, uint32_t);
extern float EEPROM_readFloat(uint32_t);

extern void EEPROM_readAll();
extern void EEPROM_writeAll();

#endif
