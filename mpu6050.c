/*
 * mpu6050.c
 * Driver MPU6050 untuk STM32 HAL via I2C
 * Taruh di: Core/Src/mpu6050.c
 */

#include "mpu6050.h"
#include <math.h>
#include <string.h>

/* Bobot gyro pada Complementary Filter (0.95 - 0.99) */
#define COMP_ALPHA   0.98f

/* Koefisien Low Pass Filter accelerometer (semakin kecil semakin halus) */
#define LPF_ALPHA    0.15f

/* Batas atas dt agar integrasi gyro tidak melompat (detik) */
#define DT_MAX       0.1f

/* Nilai WHO_AM_I yang diterima.
 * 0x68 = MPU6050 asli, 0x70/0x72 = beberapa modul clone yang umum dijual */
static uint8_t MPU6050_IsValidId(uint8_t id)
{
    return (id == 0x68 || id == 0x70 || id == 0x72);
}

/* Bungkus selisih sudut ke rentang [-180, 180] */
static float MPU6050_WrapAngle(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* Write Register */
static HAL_StatusTypeDef MPU6050_WriteReg(
    I2C_HandleTypeDef *hi2c,
    uint8_t reg,
    uint8_t val)
{
    return HAL_I2C_Mem_Write(
        hi2c,
        MPU6050_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &val,
        1,
        MPU6050_I2C_TIMEOUT
    );
}

/* Read Register */
static HAL_StatusTypeDef MPU6050_ReadRegs(
    I2C_HandleTypeDef *hi2c,
    uint8_t reg,
    uint8_t *buf,
    uint16_t len)
{
    return HAL_I2C_Mem_Read(
        hi2c,
        MPU6050_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        len,
        MPU6050_I2C_TIMEOUT
    );
}

/* MPU6050 Initialization */
HAL_StatusTypeDef MPU6050_Init(
    I2C_HandleTypeDef *hi2c,
    MPU6050_t *dev)
{
    HAL_StatusTypeDef st;

    if (hi2c == NULL || dev == NULL)
        return HAL_ERROR;

    /* Bersihkan seluruh struct (raw, filtered, offset, status) */
    memset(dev, 0, sizeof(MPU6050_t));

    /* Check device */
    st = HAL_I2C_IsDeviceReady(
        hi2c,
        MPU6050_ADDR,
        3,
        MPU6050_I2C_TIMEOUT
    );

    if (st != HAL_OK)
        return st;

    /* Check WHO_AM_I */
    st = MPU6050_ReadRegs(
        hi2c,
        MPU6050_REG_WHO_AM_I,
        &dev->who_am_i,
        1
    );

    if (st != HAL_OK)
        return st;

    if (!MPU6050_IsValidId(dev->who_am_i))
        return HAL_ERROR;

    /* Reset MPU6050 */
    st = MPU6050_WriteReg(
        hi2c,
        MPU6050_REG_PWR_MGMT_1,
        0x80
    );

    if (st != HAL_OK)
        return st;

    HAL_Delay(100);

    /* Wake up sensor, clock source = PLL gyro X */
    st = MPU6050_WriteReg(
        hi2c,
        MPU6050_REG_PWR_MGMT_1,
        0x01
    );

    if (st != HAL_OK)
        return st;

    HAL_Delay(10);

    /* DLPF = 3 (accel 44 Hz, gyro 42 Hz) -> gyro output rate 1 kHz */
    st = MPU6050_WriteReg(
        hi2c,
        MPU6050_REG_CONFIG,
        0x03
    );

    if (st != HAL_OK)
        return st;

    /* Sample rate = 1000 / (1 + 4) = 200 Hz */
    st = MPU6050_WriteReg(
        hi2c,
        MPU6050_REG_SMPLRT_DIV,
        0x04
    );

    if (st != HAL_OK)
        return st;

    /* Gyroscope +/-250 dps */
    st = MPU6050_WriteReg(
        hi2c,
        MPU6050_REG_GYRO_CONFIG,
        0x00
    );

    if (st != HAL_OK)
        return st;

    /* Accelerometer +/-2g */
    st = MPU6050_WriteReg(
        hi2c,
        MPU6050_REG_ACCEL_CONFIG,
        0x00
    );

    if (st != HAL_OK)
        return st;

    dev->last_tick = HAL_GetTick();

    return HAL_OK;
}

/* Read All Sensor Data */
HAL_StatusTypeDef MPU6050_Read_All(
    I2C_HandleTypeDef *hi2c,
    MPU6050_t *dev)
{
    uint8_t buf[14];

    HAL_StatusTypeDef st = MPU6050_ReadRegs(
        hi2c,
        MPU6050_REG_ACCEL_XOUT_H,
        buf,
        14
    );

    if (st != HAL_OK)
        return st;

    /* Raw data */
    dev->ax_raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    dev->ay_raw = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    dev->az_raw = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

    dev->temp_raw = (int16_t)(((uint16_t)buf[6] << 8) | buf[7]);

    dev->gx_raw = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    dev->gy_raw = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    dev->gz_raw = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    /* Convert accelerometer to g */
    dev->ax = (float)dev->ax_raw / MPU6050_ACCEL_SCALE;
    dev->ay = (float)dev->ay_raw / MPU6050_ACCEL_SCALE;
    dev->az = (float)dev->az_raw / MPU6050_ACCEL_SCALE;

    /* Convert gyroscope to deg/s, kurangi offset kalibrasi */
    dev->gx = (float)dev->gx_raw / MPU6050_GYRO_SCALE - dev->gx_off;
    dev->gy = (float)dev->gy_raw / MPU6050_GYRO_SCALE - dev->gy_off;
    dev->gz = (float)dev->gz_raw / MPU6050_GYRO_SCALE - dev->gz_off;

    /* Temperature (C) */
    dev->temp = (float)dev->temp_raw / 340.0f + 36.53f;

    return HAL_OK;
}

/* Gyroscope Calibration (sensor harus diam) */
HAL_StatusTypeDef MPU6050_Calibrate_Gyro(
    I2C_HandleTypeDef *hi2c,
    MPU6050_t *dev,
    uint16_t samples)
{
    float sx = 0.0f;
    float sy = 0.0f;
    float sz = 0.0f;

    uint16_t ok = 0;

    if (samples == 0)
        return HAL_ERROR;

    /* Offset di-nol-kan dulu supaya yang dirata-rata adalah nilai gyro murni */
    dev->gx_off = 0.0f;
    dev->gy_off = 0.0f;
    dev->gz_off = 0.0f;

    for (uint16_t i = 0; i < samples; i++)
    {
        if (MPU6050_Read_All(hi2c, dev) == HAL_OK)
        {
            sx += dev->gx;
            sy += dev->gy;
            sz += dev->gz;

            ok++;
        }

        HAL_Delay(5);
    }

    /* Minta minimal separuh sampel berhasil dibaca agar offset dapat dipercaya */
    if (ok < (samples / 2U) || ok == 0)
        return HAL_ERROR;

    dev->gx_off = sx / (float)ok;
    dev->gy_off = sy / (float)ok;
    dev->gz_off = sz / (float)ok;

    /* Sudut harus diinisialisasi ulang dari accelerometer setelah kalibrasi */
    dev->angle_init = 0;
    dev->last_tick = HAL_GetTick();

    return HAL_OK;
}

/* Update Roll and Pitch (Complementary Filter) */
void MPU6050_Update_Angle(MPU6050_t *dev)
{
    uint32_t now = HAL_GetTick();

    float dt = (float)(now - dev->last_tick) / 1000.0f;

    dev->last_tick = now;

    /* Ignore invalid time interval */
    if (dt <= 0.0f)
        return;

    /* Prevent large integration jumps */
    if (dt > DT_MAX)
        dt = DT_MAX;

    /* Low Pass Filter Accelerometer */
    if (!dev->angle_init)
    {
        dev->ax_filtered = dev->ax;
        dev->ay_filtered = dev->ay;
        dev->az_filtered = dev->az;
    }
    else
    {
        dev->ax_filtered += LPF_ALPHA * (dev->ax - dev->ax_filtered);
        dev->ay_filtered += LPF_ALPHA * (dev->ay - dev->ay_filtered);
        dev->az_filtered += LPF_ALPHA * (dev->az - dev->az_filtered);
    }

    /* Accelerometer angle (derajat) */
    float acc_roll = atan2f(
        dev->ay_filtered,
        dev->az_filtered
    ) * MPU6050_RAD_TO_DEG;

    float acc_pitch = atan2f(
        -dev->ax_filtered,
        sqrtf(
            dev->ay_filtered * dev->ay_filtered +
            dev->az_filtered * dev->az_filtered
        )
    ) * MPU6050_RAD_TO_DEG;

    /* First angle initialization */
    if (!dev->angle_init)
    {
        dev->roll  = acc_roll;
        dev->pitch = acc_pitch;

        dev->angle_init = 1;

        return;
    }

    /*
     * Complementary Filter dalam bentuk koreksi:
     *   angle = alpha*(angle + gyro*dt) + (1-alpha)*acc
     *         = pred + (1-alpha)*(acc - pred)
     * Selisih (acc - pred) dibungkus ke [-180, 180] supaya tidak
     * melompat saat roll melewati +/-180 derajat.
     */
    float roll_pred  = dev->roll  + dev->gx * dt;
    float pitch_pred = dev->pitch + dev->gy * dt;

    dev->roll  = MPU6050_WrapAngle(
        roll_pred + (1.0f - COMP_ALPHA) * MPU6050_WrapAngle(acc_roll - roll_pred));

    dev->pitch = MPU6050_WrapAngle(
        pitch_pred + (1.0f - COMP_ALPHA) * MPU6050_WrapAngle(acc_pitch - pitch_pred));
}
