#ifndef KALMAN_H
#define KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Kalman filter 1 dimensi (state: sudut + bias gyro) */
typedef struct
{
    float Q_angle;    // Process noise variance untuk sudut
    float Q_bias;     // Process noise variance untuk bias gyro
    float R_measure;  // Measurement noise variance (akselerometer)

    float angle;      // Sudut hasil filter (derajat)
    float bias;       // Bias gyro hasil estimasi (deg/s)
    float rate;       // Laju sudut tanpa bias (deg/s)

    float P[2][2];    // Matriks kovarians error
} Kalman_t;

void  Kalman_Init(Kalman_t *k);
void  Kalman_SetAngle(Kalman_t *k, float angle);
float Kalman_GetAngle(Kalman_t *k, float newAngle, float newRate, float dt);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_H */
