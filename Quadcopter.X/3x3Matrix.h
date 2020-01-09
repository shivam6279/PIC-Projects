#ifndef _3X3MATRIX_H_
#define _3X3MATRIX_H_

void matrix_3x3_add(float a[][3], float b[][3], float c[][3]);
void matrix_3x3_subtract(float a[][3], float b[][3], float c[][3]);
void matrix_3x3_scale(float a[][3], float s, float b[][3]);
void matrix_3x3_elementwise_multiply(float a[][3], float b[][3], float c[][3]);
void matrix_3x3_multiply(float a[][3], float b[][3], float c[][3]);
float matrix_3x3_det(float a[][3]);
void matrix_3x3_transpose(float a[][3], float b[][3]);
//float matrix_3x3_inverse(float a[][3], float b[][3]);

#endif