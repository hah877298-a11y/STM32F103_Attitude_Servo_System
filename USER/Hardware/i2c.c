#include "i2c.h"

/**
 * @file    i2c.c
 * @brief   Software-emulated I2C master (bit-banged GPIO).
 * @note    PB10 = SCL, PB11 = SDA, open-drain with 4.7k pull-ups.
 *          Slaves: MPU6050 (0x68), SSD1306 (0x3C).
 *          Half-period delay ~5 us -> ~100 kHz (standard mode).
 */

/** @brief I2C half-period delay (~5 us @ 72 MHz, ~100 kHz SCL). */
static void I2C_Delay(void)
{
    /* If the bus is unstable, raise to 200 iterations (~25 kHz). */
    for (volatile uint16_t i = 0; i < 50; i++)
    {
        __NOP();
    }
}

/**
 * @brief  Initialize I2C pins (PB10/PB11) as open-drain outputs.
 * @note   Open-drain is mandatory: it provides wired-AND arbitration
 *         and lets slaves drive SDA low for ACK.
 */
void SoftI2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(I2C_RCC_CLOCK, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_PORT, &GPIO_InitStructure);

    /* Release the bus: idle state SCL = 1, SDA = 1 */
    I2C_SCL_H();
    I2C_SDA_H();
}

/** @brief Generate I2C start condition (SDA falling edge while SCL high). */
void I2C_Start(void)
{
    I2C_SDA_H();
    I2C_SCL_H();
    I2C_Delay();

    I2C_SDA_L();
    I2C_Delay();

    I2C_SCL_L();
    I2C_Delay();
}

/** @brief Generate I2C stop condition (SDA rising edge while SCL high). */
void I2C_Stop(void)
{
    I2C_SDA_L();
    I2C_Delay();

    I2C_SCL_H();
    I2C_Delay();

    I2C_SDA_H();
    I2C_Delay();
}

/**
 * @brief  Transmit one byte, MSB first.
 * @param  data: byte to send
 */
void I2C_SendByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)        /* bit7 = 1 */
            I2C_SDA_H();
        else
            I2C_SDA_L();

        data <<= 1;

        I2C_Delay();
        I2C_SCL_H();            /* slave samples on SCL rising edge */
        I2C_Delay();
        I2C_SCL_L();
        I2C_Delay();
    }
}

/**
 * @brief  Wait for slave ACK on the 9th SCL clock.
 * @return 0 = ACK received, 1 = NAK (device absent, wrong address, ...)
 */
uint8_t I2C_WaitAck(void)
{
    uint8_t ack;

    /* Release SDA so the slave can pull it low for ACK */
    I2C_SDA_H();
    I2C_Delay();

    I2C_SCL_H();
    I2C_Delay();

    ack = I2C_SDA_READ();

    I2C_SCL_L();
    I2C_Delay();

    return ack;
}

/**
 * @brief  Receive one byte, MSB first.
 * @param  ack: ACK to send after the byte (0 = ACK, 1 = NAK).
 *              Send NAK after the final byte of a read sequence.
 * @return received byte
 */
uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0;

    /* Release SDA so the slave can drive the data line */
    I2C_SDA_H();
    I2C_Delay();

    for (i = 0; i < 8; i++)
    {
        data <<= 1;

        I2C_SCL_H();
        I2C_Delay();

        if (I2C_SDA_READ())     /* sample data while SCL high */
            data |= 0x01;

        I2C_SCL_L();
        I2C_Delay();
    }

    /* 9th clock: send ACK (continue) or NAK (last byte) */
    if (ack)
        I2C_SDA_H();
    else
        I2C_SDA_L();

    I2C_Delay();
    I2C_SCL_H();
    I2C_Delay();
    I2C_SCL_L();
    I2C_Delay();

    return data;
}
