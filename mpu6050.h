/*
 * mpu6050.h
 * Driver MPU6050 untuk STM32 (HAL) via I2C
 * Taruh di: Core/Inc/mpu6050.h
 */

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <math.h>

/* Alamat I2C MPU6050
 * AD0 = GND -> 0x68
 * AD0 = VCC -> 0x69
 *
 * HAL menggunakan alamat 8-bit (7-bit address << 1)
 */
#define MPU6050_ADDR              (0x68 << 1)

/* Register MPU6050 */
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75

/* Skala sensor
 * Accelerometer : +/- 2g      -> 16384 LSB/g
 * Gyroscope     : +/- 250 dps -> 131 LSB/(deg/s)
 */
#define MPU6050_ACCEL_SCALE       16384.0f
#define MPU6050_GYRO_SCALE        131.0f

/* I2C timeout (ms) */
#define MPU6050_I2C_TIMEOUT       100

/* Konstanta konversi */
#define MPU6050_RAD_TO_DEG        57.2957795f

/* Struktur data MPU6050 */
typedef struct
{
    /* Data mentah accelerometer */
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;

    /* Data mentah gyroscope */
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;

    /* Data mentah temperatur */
    int16_t temp_raw;

    /* Data hasil konversi */
    float ax;       /* Accelerometer X (g) */
    float ay;       /* Accelerometer Y (g) */
    float az;       /* Accelerometer Z (g) */

    float gx;       /* Gyroscope X (deg/s), sudah dikurangi offset */
    float gy;       /* Gyroscope Y (deg/s), sudah dikurangi offset */
    float gz;       /* Gyroscope Z (deg/s), sudah dikurangi offset */

    float temp;     /* Temperature (C) */

    /* Low Pass Filter accelerometer */
    float ax_filtered;
    float ay_filtered;
    float az_filtered;

    /* Sudut hasil Complementary Filter bawaan driver */
    float roll;     /* Roll (degree) */
    float pitch;    /* Pitch (degree) */

    /* Offset kalibrasi gyroscope (deg/s) */
    float gx_off;
    float gy_off;
    float gz_off;

    /* Identifikasi sensor */
    uint8_t who_am_i;

    /* Waktu pembacaan terakhir (ms, dari HAL_GetTick) */
    uint32_t last_tick;

    /* Status inisialisasi sudut (0 = belum, 1 = sudah) */
    uint8_t angle_init;

} MPU6050_t;


/* Inisialisasi MPU6050 */
HAL_StatusTypeDef MPU6050_Init(
    I2C_HandleTypeDef *hi2c,
    MPU6050_t *dev
);

/* Membaca accelerometer, gyroscope dan temperatur
 * Hasil gx/gy/gz sudah dikurangi gx_off/gy_off/gz_off */
HAL_StatusTypeDef MPU6050_Read_All(
    I2C_HandleTypeDef *hi2c,
    MPU6050_t *dev
);

/* Kalibrasi offset gyroscope
 * Sensor harus diam selama kalibrasi
 */
HAL_StatusTypeDef MPU6050_Calibrate_Gyro(
    I2C_HandleTypeDef *hi2c,
    MPU6050_t *dev,
    uint16_t samples
);

/* Menghitung Roll dan Pitch menggunakan Complementary Filter
 * Panggil setelah MPU6050_Read_All berhasil
 */
void MPU6050_Update_Angle(MPU6050_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
