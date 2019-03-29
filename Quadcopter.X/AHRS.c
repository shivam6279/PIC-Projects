#include "AHRS.h"
#include "10DOF.h"
#include "GPS.h"
#include <math.h>

void MultiplyVectorQuarternion(float q[4], XYZ r, XYZ *v) {
    float num1 = q[0] * 2.0;
	float num2 = q[1] * 2.0;
	float num3 = q[2] * 2.0;
	float num4 = q[0] * num1;
	float num5 = q[1] * num2;
	float num6 = q[2] * num3;
	float num7 = q[0] * num2;
	float num8 = q[0] * num3;
	float num9 = q[1] * num3;
	float num10 = q[3] * num1;
	float num11 = q[3] * num2;
	float num12 = q[3] * num3;
    
    v->x = (1.0 - (num5 + num6)) * r.x + (num7 - num12) * r.y + (num8 + num11) * r.z;
    v->y = (num7 + num12) * r.x + (1.0 - (num4 + num6)) * r.y + (num9 - num10) * r.z;
    v->z = (num8 - num11) * r.x + (num9 + num10) * r.y + (1.0 - (num4 + num5)) * r.z;
}

void RotateVector(float roll, float pitch, float yaw, XYZ *v) {
    float rcos = cos(roll / RAD_TO_DEGREES);
    float rsin = sin(roll / RAD_TO_DEGREES);
    
    float pcos = cos(pitch / RAD_TO_DEGREES);
    float psin = sin(pitch / RAD_TO_DEGREES);
    
    float ycos = cos(yaw / RAD_TO_DEGREES);
    float ysin = sin(yaw / RAD_TO_DEGREES);
    
    float a1 = ycos * pcos;
    float a2 = ycos * psin * rsin - ysin * pcos;
    float a3 = ycos * psin * rcos + ysin * psin;
    float b1 = ysin * pcos;
    float b2 = ysin * psin * rsin + ycos * pcos;
    float b3 = ysin * psin * rcos - ycos * psin;
    float c1 = -psin;
    float c2 = pcos * rsin;
    float c3 = pcos * rcos;
    
    XYZ temp;
    temp.x = v->x;
    temp.y = v->y;
    temp.z = v->z;
    
    v->x = temp.x * a1 + temp.y * a2 + temp.z * a3;
    v->y = temp.x * b1 + temp.y * b2 + temp.z * b3;
    v->z = temp.x * c1 + temp.y * c2 + temp.z * c3;
}

void MatrixInit(float *a) {
    
}

void MadgwickQuaternionUpdate(float q[], XYZ acc, XYZ gy, XYZ mag, float deltat) {
    float q1 = q[0];
    float q2 = q[1];
    float q3 = q[2];
    float q4 = q[3]; 
    XYZ a, g, m;
    float norm, hx, hy, _2bx, _2bz, s1, s2, s3, s4, qDot1, qDot2, qDot3, qDot4, _2q1mx, _2q1my, _2q1mz, _2q2mx, _4bx, _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;
    
    norm = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z); 
    a.x = acc.x / norm; 
    a.y = acc.y / norm; 
    a.z = acc.z / norm;
    
    g.x = gy.x / RAD_TO_DEGREES; 
    g.y = gy.y / RAD_TO_DEGREES; 
    g.z = gy.z / RAD_TO_DEGREES; 
    
    norm = sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
    m.x = mag.x / norm; 
    m.y = mag.y / norm; 
    m.z = mag.z / norm;
    
    _2q1mx = 2.0f * q1 * m.x; 
    _2q1my = 2.0f * q1 * m.y;
    _2q1mz = 2.0f * q1 * m.z; 
    _2q2mx = 2.0f * q2 * m.x;
    
    hx = m.x * q1q1 - _2q1my * q4 + _2q1mz * q3 + m.x * q2q2 + _2q2 * m.y * q3 + _2q2 * m.z * q4 - m.x * q3q3 - m.x * q4q4;
    hy = _2q1mx * q4 + m.y * q1q1 - _2q1mz * q2 + _2q2mx * q3 - m.y * q2q2 + m.y * q3q3 + _2q3 * m.z * q4 - m.y * q4q4;
    _2bx = sqrt(hx * hx + hy * hy); 
    _2bz = -_2q1mx * q3 + _2q1my * q2 + m.z * q1q1 + _2q2mx * q4 - m.z * q2q2 + _2q3 * m.y * q4 - m.z * q3q3 + m.z * q4q4; _4bx = 2.0f * _2bx; _4bz = 2.0f * _2bz;
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - a.x) + _2q2 * (2.0f * q1q2 + _2q3q4 - a.y) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m.x) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m.y) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m.z);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - a.x) + _2q1 * (2.0f * q1q2 + _2q3q4 - a.y) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - a.z) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m.x) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m.y) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m.z);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - a.x) + _2q4 * (2.0f * q1q2 + _2q3q4 - a.y) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - a.z) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m.x) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m.y) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m.z);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - a.x) + _2q3 * (2.0f * q1q2 + _2q3q4 - a.y) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m.x) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m.y) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m.z);
   
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); 
    s1 /= norm; 
    s2 /= norm; 
    s3 /= norm; 
    s4 /= norm;
    
    qDot1 = 0.5f * (-q2 * g.x - q3 * g.y - q4 * g.z) - beta * s1; 
    qDot2 = 0.5f * (q1 * g.x + q3 * g.z - q4 * g.y) - beta * s2; 
    qDot3 = 0.5f * (q1 * g.y - q2 * g.z + q4 * g.x) - beta * s3; 
    qDot4 = 0.5f * (q1 * g.z + q2 * g.y - q3 * g.x) - beta * s4;
    
    q1 += qDot1 * deltat; 
    q2 += qDot2 * deltat; 
    q3 += qDot3 * deltat; 
    q4 += qDot4 * deltat;
    
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); 
    q[0] = q1 / norm; 
    q[1] = q2 / norm; 
    q[2] = q3 / norm; 
    q[3] = q4 / norm;
}