#include "AHRS.h"
#include "10DOF.h"
#include "GPS.h"
#include "PID.h"
#include <math.h>

float roll_offset = ROLLOFFSET;
float pitch_offset = PITCHOFFSET;
float heading_offset = HEADINGOFFSET;

XYZ MultiplyVectorQuaternion(XYZ v, float q_in[4]) {
    XYZ ret;
    float q[4];
    q[0] = q_in[0];
    q[1] = q_in[1];
    q[2] = q_in[2];
    q[3] = q_in[3];

    // float qw_2 = q[0]*q[0];
    // float qi_2 = q[1]*q[1];
    // float qj_2 = q]2]*q[2];
    // float qk_2 = q[3]*q[3];
    // float qwi = q[0]*q[1];
    // float qwj = q[0]*q[2];
    // float qwk = q[0]*q[3];
    // float qij = q[1]*q[2];
    // float qik = q[1]*q[3];
    // float qjk = q[2]*q[3];

    float r11 = 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]);
	float r12 = 2.0f*(q[1]*q[2] - q[3]*q[0]);
	float r13 = 2.0f*(q[1]*q[3] + q[2]*q[0]);
	float r21 = 2.0f*(q[1]*q[2] + q[3]*q[0]);
	float r22 = 1.0f - 2.0f*(q[1]*q[1] + q[3]*q[3]);
	float r23 = 2.0f*(q[2]*q[3] - q[1]*q[0]);
	float r31 = 2.0f*(q[1]*q[3] - q[2]*q[0]);
	float r32 = 2.0f*(q[2]*q[3] + q[1]*q[0]);
	float r33 = 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]);
    
    ret.x = v.x * r11 + v.y * r12 + v.z * r13;
    ret.y = v.x * r21 + v.y * r22 + v.z * r23;
    ret.z = v.x * r31 + v.y * r32 + v.z * r33;

    return ret;
}

XYZ RotateVectorEuler(XYZ v, float roll, float pitch, float yaw) {
    XYZ r;
    
    roll = TO_RAD(roll);
    pitch = TO_RAD(pitch);
    yaw = TO_RAD(yaw);

    float rcos = cos(roll);
    float rsin = sin(roll);
    
    float pcos = cos(pitch);
    float psin = sin(pitch);
    
    float ycos = cos(yaw);
    float ysin = sin(yaw);
    
    float a1 = ycos * pcos;
    float a2 = ycos * psin * rsin - ysin * pcos;
    float a3 = ycos * psin * rcos + ysin * psin;
    float b1 = ysin * pcos;
    float b2 = ysin * psin * rsin + ycos * pcos;
    float b3 = ysin * psin * rcos - ycos * psin;
    float c1 = -psin;
    float c2 = pcos * rsin;
    float c3 = pcos * rcos;

    r.x = v.x * a1 + v.y * a2 + v.z * a3;
    r.y = v.x * b1 + v.y * b2 + v.z * b3;
    r.z = v.x * c1 + v.y * c2 + v.z * c3;

    return r;
}

void QuaternionToEuler(float q[], float *roll, float *pitch, float *yaw) {
    float q2_2 = q[2] * q[2];

    //Converting quaternion to Euler angles
    *roll = 180.0f + TO_DEG(atan2(q[2]*q[3] + q[0]*q[1], 0.5f - (q[1]*q[1] + q2_2)));
    *pitch = TO_DEG(asin(2.0f * (q[1]*q[3] - q[0]*q[2])));
    *yaw = TO_DEG(atan2((q[1]*q[2] + q[0]*q[3]), 0.5f - (q2_2 + q[3]*q[3])));
    
    *roll = LimitAngle(*roll - roll_offset);
    *pitch = LimitAngle(*pitch - pitch_offset);
    *yaw = LimitAngle(*yaw - heading_offset);
}

void MultiplyQuaternion(float q1[], float q2[], float qr[]) {
    qr[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    qr[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    qr[2] = q1[0]*q2[2] + q1[2]*q2[0] + q1[3]*q2[1] - q1[1]*q2[3];
    qr[3] = q1[0]*q2[3] + q1[3]*q2[0] + q1[1]*q2[2] - q1[2]*q2[1]; 
}

XYZ GetCompensatedAcc(float q[], XYZ acc, float gravity_mag) {
    XYZ g, acc_pure;
    float q_conj[4], temp[4], ret[4];

    g.x = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    g.y = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    g.z = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    g = VectorScale(g, gravity_mag);

    acc_pure.x = acc.x - g.x;
    acc_pure.y = acc.y - g.y;
    acc_pure.z = acc.z - g.z;

    q_conj[0] =  q[0];
    q_conj[1] = -q[1];
    q_conj[2] = -q[2];
    q_conj[3] = -q[3];

    MultiplyQuaternion(q, (float[4]){0.0, acc_pure.x,  acc_pure.y,  acc_pure.z}, temp);
    MultiplyQuaternion(temp, q_conj, ret);

    return (XYZ){ret[1], ret[2], ret[3]};
}

void MadgwickQuaternionUpdate(float q[], XYZ a, XYZ g, XYZ m, float deltat) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    g.x = TO_RAD(g.x);
    g.y = TO_RAD(g.y);
    g.z = TO_RAD(g.z);

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
    hx = m.x * q0q0 - _2q0my * q[3] + _2q0mz * q[2] + m.x * q1q1 + _2q1 * m.y * q[2] + _2q1 * m.z * q[3] - m.x * q2q2 - m.x * q3q3;
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
}

void MadgwickQuaternionUpdateGyro(float q[], XYZ g, float deltat) {
    float recipNorm;
    float qDot1, qDot2, qDot3, qDot4;
    // Convert gyroscope degrees/sec to radians/sec
    g.x = TO_RAD(g.x);
    g.y = TO_RAD(g.y);
    g.z = TO_RAD(g.z);

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q[1] * g.x - q[2] * g.y - q[3] * g.z);
    qDot2 = 0.5f * (q[0] * g.x + q[2] * g.z - q[3] * g.y);
    qDot3 = 0.5f * (q[0] * g.y - q[1] * g.z + q[3] * g.x);
    qDot4 = 0.5f * (q[0] * g.z + q[1] * g.y - q[2] * g.x);

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
}

void MadgwickQuaternionUpdateAcc(float q[], XYZ a, float deltat) {
	float recipNorm;
	float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Normalise accelerometer measurement
    recipNorm = invSqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    a.x *= recipNorm;
    a.y *= recipNorm;
    a.z *= recipNorm;
    
    _2q0 = 2.0f * q[0];
    _2q1 = 2.0f * q[1];
    _2q2 = 2.0f * q[2];
    _2q3 = 2.0f * q[3];
    _4q0 = 4.0f * q[0];
    _4q1 = 4.0f * q[1];
    _4q2 = 4.0f * q[2];
    _8q1 = 8.0f * q[1];
    _8q2 = 8.0f * q[2];
    q0q0 = q[0] * q[0];
    q1q1 = q[1] * q[1];
    q2q2 = q[2] * q[2];
    q3q3 = q[3] * q[3];

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * a.x + _4q0 * q1q1 - _2q1 * a.y;
    s1 = _4q1 * q3q3 - _2q3 * a.x + 4.0f * q0q0 * q[1] - _2q0 * a.y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a.z;
    s2 = 4.0f * q0q0 * q[2] + _2q0 * a.x + _4q2 * q3q3 - _2q3 * a.y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a.z;
    s3 = 4.0f * q1q1 * q[3] - _2q1 * a.x + 4.0f * q2q2 * q[3] - _2q2 * a.y;

    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 = -FUSION_BETA * s0;
    qDot2 = -FUSION_BETA * s1;
    qDot3 = -FUSION_BETA * s2;
    qDot4 = -FUSION_BETA * s3;

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


static float altitude_kf_P[2][2] = { { 1.0f, 0.0f },
                                     { 0.0f, 1.0f } };
static float altitude_kf_h = 0.0;
static float altitude_kf_v = 0.0;

static float altitude_kf_acc_buffer[ALITUDE_KF_ACC_BUFFER_SIZE];

void altitude_KF_reset() {
    altitude_kf_P[0][0] = 1.0f;
    altitude_kf_P[0][1] = 0.0f;
    altitude_kf_P[1][0] = 0.0f;
    altitude_kf_P[1][1] = 1.0f;

    altitude_kf_h = 0.0;
    altitude_kf_v = 0.0;
    
    int i;
    for(i = 0; i < ALITUDE_KF_ACC_BUFFER_SIZE; i++) {
        altitude_kf_acc_buffer[i] = 0.0;
    }
}

void altitude_KF_propagate(float acc, float dt) {
    int i;
    
    for(i = (ALITUDE_KF_ACC_BUFFER_SIZE - 1); i >= 1; i--)
        altitude_kf_acc_buffer[i] = altitude_kf_acc_buffer[i - 1];

    altitude_kf_acc_buffer[0] = acc;
    
    float acceleration;
    for(i = 0, acceleration = 0.0; i < ALITUDE_KF_ACC_BUFFER_SIZE; i++)
        acceleration += altitude_kf_acc_buffer[i];
    
    acceleration /= (float)ALITUDE_KF_ACC_BUFFER_SIZE;
    
    float _dtdt = dt * dt;

    // Propagation of the state (equation of motion) by Euler integration
    altitude_kf_h += altitude_kf_v * dt + 0.5f * acceleration * _dtdt;
    altitude_kf_v += acceleration * dt;

    // Calculate the state estimate covariance
    float _Q_accel_dtdt = ALTITUDE_KF_Q * _dtdt;

    altitude_kf_P[0][0] = altitude_kf_P[0][0] + (altitude_kf_P[1][0] + altitude_kf_P[0][1] + (altitude_kf_P[1][1] + 0.25f*_Q_accel_dtdt) * dt) * dt;
    altitude_kf_P[0][1] = altitude_kf_P[0][1] + (altitude_kf_P[1][1] + 0.5f * _Q_accel_dtdt) * dt;
    altitude_kf_P[1][0] = altitude_kf_P[1][0] + (altitude_kf_P[1][1] + 0.5f * _Q_accel_dtdt) * dt;
    altitude_kf_P[1][1] = altitude_kf_P[1][1] + _Q_accel_dtdt;
}


void altitude_KF_update(float altitude) {
    float y = altitude - altitude_kf_h;
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

float altitude_KF_setAltitude(float alt) {
    altitude_kf_h = alt;
}

float altitude_KF_setVelocity(float vel) {
    altitude_kf_v = vel;
}

float altitude_KF_getAltitude() {
    return altitude_kf_h;
}

float altitude_KF_getVelocity() {
    return altitude_kf_v;
}