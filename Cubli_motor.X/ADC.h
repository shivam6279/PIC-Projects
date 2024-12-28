#ifndef _ADC_H_
#define _ADC_H_

#include <xc.h>

#define ADC_CONV_FACTOR (3.3f / 4095.0f)

#define VSNS_VBAT_DIVIDER (4.7f/(4.7f + 33.0f))
#define VSNS_12V_DIVIDER (4.7f/(4.7f + 33.0f))

#define ISNS_UVW_R 0.008f
#define ISNS_VBAT_R 0.01f

extern unsigned int adc_data[5];

extern void ADCInit();
extern void adc_readAll();

#endif