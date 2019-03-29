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

void QuaternionToEuler(float q[], float *roll, float *pitch, float *yaw) {
    roll->p_error = roll->error;
    pitch->p_error = pitch->error;
    yaw->p_error = yaw->error;

    float a = q[2] * q[2];

    /*
    *roll = (atan2(2.0f * (q[0] * q[2] - q[3] * q[1]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) + PI) * RAD_TO_DEGREES - ROLLOFFSET;
    *pitch = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) + PI) * RAD_TO_DEGREES - PITCHOFFSET;
    *heading = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * RAD_TO_DEGREES - HEADINGOFFSET;
    */

    //Converting quaternion to Euler angles
    *roll = (asin(2.0f * (q[0] * q[2] - q[3] * q[1])) + PI) * RAD_TO_DEGREES - ROLLOFFSET;
    *pitch = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + a)) + PI) * RAD_TO_DEGREES - PITCHOFFSET;
    *heading = -atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (a + q[3] * q[3])) * RAD_TO_DEGREES - HEADINGOFFSET;
    
    //Limit angles within -180 and +180 degrees
    LimitAngle(heading);
    LimitAngle(roll);
    LimitAngle(pitch);
}

void MadgwickQuaternionUpdate(float q[], XYZ a, XYZ g XYZ m, float deltat) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    g.x *= 0.0174533f;
    g.y *= 0.0174533f;
    g.z *= 0.0174533f;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q[1] * g.x - q[2] * g.y - q[3] * g.z);
    qDot2 = 0.5f * (q[0] * g.x + q[2] * g.z - q[3] * g.y);
    qDot3 = 0.5f * (q[0] * g.y - q[1] * g.z + q[3] * g.x);
    qDot4 = 0.5f * (q[0] * g.z + q[1] * g.y - q[2] * g.x);

    // Normalise accelerometer measurement
    recipNorm = invSqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    a.x *= recipNorm;
    a.y *= recipNorm;
    a.z *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(m.x * m.x + m.y * m.y + m.z * m.z);
    m.x *= recipNorm;
    m.y *= recipNorm;
    m.z *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q[0] * m.x;
    _2q0my = 2.0f * q[0] * m.y;
    _2q0mz = 2.0f * q[0] * m.z;
    _2q1mx = 2.0f * q[1] * m.x;
    _2q0 = 2.0f * q[0];
    _2q1 = 2.0f * q[1];
    _2q2 = 2.0f * q[2];
    _2q3 = 2.0f * q[3];
    _2q0q2 = 2.0f * q[0] * q[2];
    _2q2q3 = 2.0f * q[2] * q[3];
    q0q0 = q[0] * q[0];
    q0q1 = q[0] * q[1];
    q0q2 = q[0] * q[2];
    q0q3 = q[0] * q[3];
    q1q1 = q[1] * q[1];
    q1q2 = q[1] * q[2];
    q1q3 = q[1] * q[3];
    q2q2 = q[2] * q[2];
    q2q3 = q[2] * q[3];
    q3q3 = q[3] * q[3];

    // Reference direction of Earth's magnetic field
    hx = m.x * q0q0 - _2q0my * q3 + _2q0mz * q[2] + m.x * q1q1 + _2q1 * m.y * q[2] + _2q1 * m.z * q[3] - m.x * q2q2 - m.x * q3q3;
    hy = _2q0mx * q[3] + m.y * q0q0 - _2q0mz * q[1] + _2q1mx * q[2] - m.y * q1q1 + m.y * q2q2 + _2q2 * m.z * q[3] - m.y * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q[2] + _2q0my * q[1] + m.z * q0q0 + _2q1mx * q[3] - m.z * q1q1 + _2q2 * m.y * q[3] - m.z * q2q2 + m.z * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - a.x) + _2q1 * (2.0f * q0q1 + _2q2q3 - a.y) - _2bz * q[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + _2bx * q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - a.x) + _2q0 * (2.0f * q0q1 + _2q2q3 - a.y) - 4.0f * q[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a.z) + _2bz * q[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - a.x) + _2q3 * (2.0f * q0q1 + _2q2q3 - a.y) - 4.0f * q[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a.z) + (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - a.x) + _2q2 * (2.0f * q0q1 + _2q2q3 - a.y) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= FUSION_BETA * s0;
    qDot2 -= FUSION_BETA * s1;
    qDot3 -= FUSION_BETA * s2;
    qDot4 -= FUSION_BETA * s3;

    // Integrate rate of change of quaternion to yield quaternion
    q[0] += qDot1 * deltat;
    q[1] += qDot2 * deltat;
    q[2] += qDot3 * deltat;
    q[3] += qDot4 * deltat;

    // Normalise quaternion
    recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
    anglesComputed = 0;
}

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float altitude_kf_P[2][2] = { { 1.0f, 0.0f },
                              { 0.0f, 1.0f } };

float altitude_kf_h = 0.0;
float altitude_kf_v = 0.0;

void altitude_KF_reset() {
    altitude_kf_P[2][2] =
    {
        { 1.0f,     0.0f },
        { 0.0f,     1.0f }
    };

    altitude_kf_h = 0.0;
    altitude_kf_v = 0.0;
}

void altitude_KF_propagate(float acceleration, float dt) {
    float _dtdt = dt * dt;

    // Propagation of the state (equation of motion) by Euler integration
    altitude_kf_h = altitude_kf_h + altitude_kf_v * dt + 0.5f * acceleration * _dtdt;
    altitude_kf_v = altitude_kf_v + acceleration * dt;

    // Calculate the state estimate covariance
    float _Q_accel_dtdt = ALTITUDE_KF_Q * _dtdt;

    altitude_kf_P[0][0] = altitude_kf_P[0][0] + (altitude_kf_P[1][0] + altitude_kf_P[0][1] + (altitude_kf_P[1][1] + 0.25f*_Q_accel_dtdt) * dt) * dt;
    altitude_kf_P[0][1] = altitude_kf_P[0][1] + (altitude_kf_P[1][1] + 0.5f * _Q_accel_dtdt) * dt;
    altitude_kf_P[1][0] = altitude_kf_P[1][0] + (altitude_kf_P[1][1] + 0.5f * _Q_accel_dtdt) * dt;
    altitude_kf_P[1][1] = altitude_kf_P[1][1] + _Q_accel_dtdt;
}


void altitude_KF_update(float altitude) {
    float y = altitude - h;
    float Sinv = 1.0f / (altitude_kf_P[0][0] + ALTITUDE_KF_R);

    // Calculate the Kalman gain
    float K[2] = { altitude_kf_P[0][0] * Sinv, altitude_kf_P[1][0] * Sinv };

    // Update the state estimate
    altitude_kf_h += K[0] * y;
    altitude_kf_v += K[1] * y;

    // Calculate the state estimate covariance
    altitude_kf_P[0][0] = altitude_kf_P[0][0] - K[0] * altitude_kf_P[0][0];
    altitude_kf_P[0][1] = altitude_kf_P[0][1] - K[0] * altitude_kf_P[0][1];
    altitude_kf_P[1][0] = altitude_kf_P[1][0] - (K[1] * altitude_kf_P[0][0]);
    altitude_kf_P[1][1] = altitude_kf_P[1][1] - (K[1] * altitude_kf_P[0][1]);
}

float altitude_KF_getAltitude() {
    return altitude_kf_h;
}

float altitude_KF_getVelocity() {
    return altitude_kf_v;
}