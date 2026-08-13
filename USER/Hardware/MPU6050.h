#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

/**
 * @file    MPU6050.h
 * @brief   MPU6050 6-axis IMU driver (I2C).
 * @note    VCC 3.3V, GND, SCL PB10, SDA PB11, AD0 = GND (address 0x68).
 *          Requires software I2C (i2c.h).
 */

/**
 * @name  I2C addresses
 * @note  AD0 = GND -> 7-bit address 0x68 (AD0 = VCC -> 0x69).
 *        Bus byte = (addr << 1) | R/W.
 */
#define MPU6050_ADDR            0x68   /* 7-bit address (AD0 = GND) */
#define MPU6050_ADDR_WRITE      (MPU6050_ADDR << 1)       /* 0xD0 */
#define MPU6050_ADDR_READ       (MPU6050_ADDR << 1 | 1)   /* 0xD1 */

/* ---- register map ---- */
/* Registers used by this driver only; full list in the datasheet. */

#define MPU6050_REG_SMPLRT_DIV   0x19   /* sample rate divider */
#define MPU6050_REG_CONFIG       0x1A   /* DLPF bandwidth */
#define MPU6050_REG_GYRO_CONFIG  0x1B   /* gyro full-scale */
#define MPU6050_REG_ACCEL_CONFIG 0x1C   /* accel full-scale */

/* data block starts at 0x3B: 14 consecutive bytes */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B   /* accel X high byte */

#define MPU6050_REG_PWR_MGMT_1   0x6B   /* sleep/wake, clock source */
#define MPU6050_REG_WHO_AM_I     0x75   /* device ID (0x68) */

/* ---- full-scale conversion factors ---- */
/* Raw values are signed 16-bit; divide by the factor of the configured
 * full-scale range to obtain physical units. */
#define MPU6050_GYRO_SCALE       16.4f    /* +/-2000 deg/s: 32768/2000 LSB/(deg/s) */
#define MPU6050_ACCEL_SCALE      2048.0f  /* +/-16 g: 32768/16 LSB/g */

/* ---- data structures ---- */
/** @brief Raw sensor data (signed 16-bit, read from registers). */
typedef struct
{
    int16_t Accel_X;    /* accel X raw */
    int16_t Accel_Y;    /* accel Y raw */
    int16_t Accel_Z;    /* accel Z raw */
    int16_t Temp;       /* temperature raw */
    int16_t Gyro_X;     /* gyro X raw */
    int16_t Gyro_Y;     /* gyro Y raw */
    int16_t Gyro_Z;     /* gyro Z raw */
} MPU6050_RawData;

/** @brief Physical units: accel g, gyro deg/s, temperature degC. */
typedef struct
{
    float Accel_X_g;    /* accel X (g) */
    float Accel_Y_g;    /* accel Y (g) */
    float Accel_Z_g;    /* accel Z (g) */
    float Gyro_X_dps;   /* gyro X (deg/s) */
    float Gyro_Y_dps;   /* gyro Y (deg/s) */
    float Gyro_Z_dps;   /* gyro Z (deg/s) */
    float Temp_C;       /* temperature (degC) */
} MPU6050_PhyData;

/* ---- public API ---- */

/** @brief Initialize: wake-up, sample rate, DLPF, full-scale ranges. */
void MPU6050_Init(void);

/** @brief Read WHO_AM_I; @return 0x68 when communication is OK. */
uint8_t MPU6050_ReadID(void);

/** @brief Read all raw sensor channels in one transaction. */
void MPU6050_ReadRawData(MPU6050_RawData *data);
/** @brief Convert raw values to physical units. */
void MPU6050_ConvertToPhy(const MPU6050_RawData *raw, MPU6050_PhyData *phy);

/* low-level register access */
/** @brief Write one register. */
void MPU6050_WriteReg(uint8_t reg, uint8_t value);
/** @brief Read multiple consecutive registers. */
void MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);

#endif /* __MPU6050_H */
