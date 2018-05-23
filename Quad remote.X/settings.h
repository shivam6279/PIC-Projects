#ifndef _settings_H_
#define _settings_H_

// x1_pin AN12 - RG8
// x2_pin AN9 - RB14
// y1_pin AN13 - RG7
// y2_pin AN10 - RB15
//#define dail1_pin
//#define dail2_pin
#define x1_offset 20350.0
#define x1_min 0.0
#define x1_max 40950.0
#define y1_offset 20400.0
#define y1_min 0.0
#define y1_max 40950.0
#define x2_offset 19900.0
#define x2_min 0.0
#define x2_max 40950.0
#define y2_min 0.0
#define y2_max 40320.0

// ts_xl AN18 - E4
// ts_x2 AN15 - E7
// ts_y1 AN17 - E5
// ts_y2 AN16 - E6
#define ts_x_min 614
#define ts_x_max 3436
#define ts_y_min 390
#define ts_y_max 3116

#define switch1_pin PORTBbits.RB6
#define switch2_pin PORTBbits.RB13

#define LCD_backlight_pin PORTGbits.RG6
#define speaker_pin LATCbits.LATC14

#endif