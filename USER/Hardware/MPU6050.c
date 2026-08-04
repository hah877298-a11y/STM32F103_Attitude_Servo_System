#include "MPU6050.h"
#include "i2c.h"

/**
 * @file    MPU6050.c
 * @brief   MPU6050 6-axis IMU driver (I2C).
 * @note    3-axis accel (configured +/-16 g) + 3-axis gyro (+/-2000 deg/s)
 *          + temperature. Register access via soft I2C (i2c.h):
 *          write [S][A+W][reg][data][P], read [S][A+W][reg][S][A+R][data][P].
 */

/**
 * @brief  Write one byte to a register.
 * @param  reg:   register address (0x00..0x7F)
 * @param  value: value to write
 */
void MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    I2C_Start();

    I2C_SendByte(MPU6050_ADDR_WRITE);            /* addr + write */
    I2C_WaitAck();

    I2C_SendByte(reg);                           /* target register */
    I2C_WaitAck();

    I2C_SendByte(value);                         /* data */
    I2C_WaitAck();

    I2C_Stop();
}

/**
 * @brief  Read multiple bytes starting at reg (address auto-increments).
 * @param  reg: start register address
 * @param  buf: destination buffer
 * @param  len: number of bytes to read
 * @note   The final byte is acknowledged with NAK, per I2C protocol.
 */
void MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    /* write mode: set the internal register pointer */
    I2C_Start();
    I2C_SendByte(MPU6050_ADDR_WRITE);
    I2C_WaitAck();
    I2C_SendByte(reg);
    I2C_WaitAck();

    /* repeated start, switch to read mode */
    I2C_Start();
    I2C_SendByte(MPU6050_ADDR_READ);
    I2C_WaitAck();

    /* ACK all bytes except the last (NAK terminates the read) */
    for (i = 0; i < len; i++)
    {
        buf[i] = I2C_ReadByte(i == len - 1 ? 1 : 0);
    }

    I2C_Stop();
}

/**
 * @brief  Initialize MPU6050: wake-up, 125 Hz output rate, DLPF 5 Hz,
 *         gyro +/-2000 deg/s, accel +/-16 g.
 */
void MPU6050_Init(void)
{
    /* wake up: clear SLEEP bit of PWR_MGMT_1 (0x6B), use internal clock */
    MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00);

    /* sample rate = 1 kHz / (1 + 7) = 125 Hz */
    MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x07);

    /* DLPF: gyro bandwidth ~5 Hz (smooth output) */
    MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x06);

    /* gyro full-scale: +/-2000 deg/s (bit4:3 = 11) */
    MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x18);

    /* accel full-scale: +/-16 g (bit4:3 = 11) */
    MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x18);
}

/**
 * @brief  Read the WHO_AM_I register to verify I2C communication.
 * @return register value (0x68 when the bus and address are correct)
 * @note   Any other value indicates wiring, power, address or pull-up
 *         problems (e.g. SDA/SCL swapped, missing 4.7k pull-up, AD0 not
 *         tied to GND).
 */
uint8_t MPU6050_ReadID(void)
{
    uint8_t id;

    MPU6050_ReadRegs(MPU6050_REG_WHO_AM_I, &id, 1);

    return id;
}

/**
 * @brief  Read all 14 sensor bytes in one transaction (same sample
 *         instant, auto-increment addressing).
 * @param  data: raw data structure to fill
 * @note   0x3B..0x48: AX, AY, AZ, T, GX, GY, GZ as H/L byte pairs,
 *          big-endian, signed 16-bit.
 */
void MPU6050_ReadRawData(MPU6050_RawData *data)
{
    uint8_t buf[14];

    /* 14 bytes starting at ACCEL_XOUT_H (0x3B) */
    MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, buf, 14);

    /* merge H/L byte pairs into signed 16-bit values */
    data->Accel_X = (int16_t)((buf[0]  << 8) | buf[1]);   /* buf[0]=H, buf[1]=L */
    data->Accel_Y = (int16_t)((buf[2]  << 8) | buf[3]);
    data->Accel_Z = (int16_t)((buf[4]  << 8) | buf[5]);
    data->Temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    data->Gyro_X  = (int16_t)((buf[8]  << 8) | buf[9]);
    data->Gyro_Y  = (int16_t)((buf[10] << 8) | buf[11]);
    data->Gyro_Z  = (int16_t)((buf[12] << 8) | buf[13]);
}

/**
 * @brief  Convert raw sensor values to physical units.
 * @param  raw: raw sensor data
 * @param  phy: converted values (accel in g, gyro in deg/s, temp in degC)
 * @note   temp (degC) = raw / 340.0 + 36.53; accel/gyro divide by the
 *         LSB-per-unit factor of the configured full-scale ranges.
 */
void MPU6050_ConvertToPhy(const MPU6050_RawData *raw, MPU6050_PhyData *phy)
{
    /* accel: raw / LSB-per-g factor */
    phy->Accel_X_g = raw->Accel_X / MPU6050_ACCEL_SCALE;
    phy->Accel_Y_g = raw->Accel_Y / MPU6050_ACCEL_SCALE;
    phy->Accel_Z_g = raw->Accel_Z / MPU6050_ACCEL_SCALE;

    /* gyro: raw / LSB-per-(deg/s) factor */
    phy->Gyro_X_dps = raw->Gyro_X / MPU6050_GYRO_SCALE;
    phy->Gyro_Y_dps = raw->Gyro_Y / MPU6050_GYRO_SCALE;
    phy->Gyro_Z_dps = raw->Gyro_Z / MPU6050_GYRO_SCALE;

    /* temp: sensitivity 340 LSB/degC, offset 36.53 degC */
    phy->Temp_C = raw->Temp / 340.0f + 36.53f;
}
