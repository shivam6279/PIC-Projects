#include "ToF.h"
#include "bitbang_I2C.h"

#define VL6180X_ADDRESS 0x52

void VL6180_init() {
    // private settings from page 24 of app note
    VL6180_write8(0x0207, 0x01);
    VL6180_write8(0x0208, 0x01);
    VL6180_write8(0x0096, 0x00);
    VL6180_write8(0x0097, 0xfd);
    VL6180_write8(0x00e3, 0x00);
    VL6180_write8(0x00e4, 0x04);
    VL6180_write8(0x00e5, 0x02);
    VL6180_write8(0x00e6, 0x01);
    VL6180_write8(0x00e7, 0x03);
    VL6180_write8(0x00f5, 0x02);
    VL6180_write8(0x00d9, 0x05);
    VL6180_write8(0x00db, 0xce);
    VL6180_write8(0x00dc, 0x03);
    VL6180_write8(0x00dd, 0xf8);
    VL6180_write8(0x009f, 0x00);
    VL6180_write8(0x00a3, 0x3c);
    VL6180_write8(0x00b7, 0x00);
    VL6180_write8(0x00bb, 0x3c);
    VL6180_write8(0x00b2, 0x09);
    VL6180_write8(0x00ca, 0x09);
    VL6180_write8(0x0198, 0x01);
    VL6180_write8(0x01b0, 0x17);
    VL6180_write8(0x01ad, 0x00);
    VL6180_write8(0x00ff, 0x05);
    VL6180_write8(0x0100, 0x05);
    VL6180_write8(0x0199, 0x05);
    VL6180_write8(0x01a6, 0x1b);
    VL6180_write8(0x01ac, 0x3e);
    VL6180_write8(0x01a7, 0x1f);
    VL6180_write8(0x0030, 0x00);

    VL6180_write8(0x0011, 0x10);    // Enables polling for 'New Sample ready'
                                    // when measurement completes
    VL6180_write8(0x010a, 0x30);    // Set the averaging sample period
                                    // (compromise between lower noise and
                                    // increased execution time)
    VL6180_write8(0x003f, 0x46);    // Sets the light and dark gain (upper
                                    // nibble). Dark gain should not be
                                    // changed.
    VL6180_write8(0x0031, 0xFF);    // sets the # of range measurements after
                                    // which auto calibration of system is
                                    // performed
    VL6180_write8(0x0040, 0x63);    // Set ALS integration time to 100ms
    VL6180_write8(0x002e, 0x01);    // perform a single temperature calibration
                                    // of the ranging sensor

    // Optional: Public registers - See data sheet for more detail
    VL6180_write8(0x001b, 0x09);    // Set default ranging inter-measurement
                                    // period to 100ms
    VL6180_write8(0x003e, 0x31);    // Set default ALS inter-measurement period
                                    // to 500ms
    VL6180_write8(0x0014, 0x24);    // Configures interrupt on 'New Sample
    // Ready threshold event'

    VL6180_write8(0x016, 0x00);
}

unsigned char ToF_readRange() {
  // wait for device to be ready for range measurement
  while (! (VL6180_read8(0x04D) & 0x01));
  // Start a range measurement
  VL6180_write8(0x018, 0x01);
  // Poll until bit 2 is set
  while (! (VL6180_read8(0x04F) & 0x04));
  // read range in mm
  unsigned char range = VL6180_read8(0x062);
  // clear interrupt
  VL6180_write8(0x015, 0x07);
  return range;
}

unsigned char ToF_valueGood() {
  return (VL6180_read8(0x04D) >> 4);
}

unsigned char VL6180_read8(unsigned int addr) {
    unsigned char data;
    I2C_Start();
    I2C_Send(VL6180X_ADDRESS & 0xFE); 
    I2C_GetAck();
    I2C_Send(addr >> 8);
    I2C_GetAck();
    I2C_Send(addr & 0xFF);
    I2C_GetAck();
    I2C_Start();
    I2C_Send(VL6180X_ADDRESS | 0x01); 
    I2C_GetAck();
    data = I2C_Read();
    I2C_SendNak();
    I2C_Stop();

    return data;
}

void VL6180_write8(unsigned int addr, unsigned char data) {
    I2C_WriteRegisters(VL6180X_ADDRESS, (unsigned char[3]){addr >> 8, addr & 0xFF, data}, 3);
}