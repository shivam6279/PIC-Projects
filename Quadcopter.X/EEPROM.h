#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <stdbool.h>
#include "PID.h"

bool eeprom_writeByte(unsigned char, unsigned char);
unsigned char eeprom_readByte(unsigned char);
bool eeprom_writeBytes(unsigned char, unsigned char*, unsigned char);
unsigned char eeprom_readBytes(unsigned char, unsigned char*, unsigned char);
void eeprom_readPID(PID*, PID*, PID*, PID*, PID*);
void eeprom_writePID(PID*, PID*, PID*, PID*, PID*);

#endif