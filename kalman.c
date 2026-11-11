#include "kalman.h"

void Kalman_Init(Kalman_t *k)
{
    k->Q_angle   = 0.001f;
    k->Q_bias    = 0.003f;
    k->R_measure = 0.03f;

    k->angle = 0.0f;
    k->bias  = 0.0f;
    k->rate  = 0.0f;

    k->P[0][0] = 0.0f;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 0.0f;
}

void Kalman_SetAngle(Kalman_t *k, float angle)
{
    k->angle = angle;
}

/*
 * newAngle : sudut dari akselerometer (derajat)
 * newRate  : laju sudut dari gyro (deg/s)
 * dt       : selang waktu antar panggilan (detik)
 * return   : sudut hasil filter (derajat)
 */
float Kalman_GetAngle(Kalman_t *k, float newAngle, float newRate, float dt)
{
    /* --- Prediksi --- */
    k->rate   = newRate - k->bias;
    k->angle += dt * k->rate;

    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    /* --- Update --- */
    float S  = k->P[0][0] + k->R_measure;   // Innovation covariance
    float K0 = k->P[0][0] / S;              // Kalman gain
    float K1 = k->P[1][0] / S;

    float y = newAngle - k->angle;          // Innovation

    k->angle += K0 * y;
    k->bias  += K1 * y;

    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];

    k->P[0][0] -= K0 * P00_temp;
    k->P[0][1] -= K0 * P01_temp;
    k->P[1][0] -= K1 * P00_temp;
    k->P[1][1] -= K1 * P01_temp;

    return k->angle;
}
