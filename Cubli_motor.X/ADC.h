#ifndef _ADC_H_
#define _ADC_H_

#include <xc.h>
#include <inttypes.h>

#define ADC_CONV_FACTOR (3.3f / 4095.0f)

#define VSNS_VBAT_DIVIDER (4.7f/(4.7f + 33.0f))
#define VSNS_12V_DIVIDER (10.0f/(10.0f + 33.0f))

#define MOTOR_VSNS_DIVIDER (10.0f / (33.0f + 10.0f))

#define ISNS_UVW_R 0.005f
#define ISNS_VBAT_R 0.005f

#define ADC_BUFFER_LEN 1

extern unsigned int adc_data[49];
extern volatile uint16_t adc_buffer[6][2][ADC_BUFFER_LEN];
extern volatile uint8_t adc_cnt_buffer[6][2];

extern volatile float isns_u_offset, isns_v_offset;

extern void ADCCalib();
extern void ADCInit();
extern void adc_readAll();

#endif
