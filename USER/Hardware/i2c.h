#ifndef __I2C_H
#define __I2C_H

#include "stm32f10x.h"

/**
 * @file    i2c.h
 * @brief   Software-emulated I2C master (bit-banged GPIO).
 * @note    PB10 = SCL, PB11 = SDA, each with 4.7k pull-up to 3.3 V.
 *          Open-drain pins allow sharing the bus between slaves with
 *          distinct 7-bit addresses (MPU6050, SSD1306).
 */

/* ========== 引脚宏定义 (方便修改) ========== */
#define I2C_SCL_PIN       GPIO_Pin_10    /* PB10 → SCL 时钟线 */
#define I2C_SDA_PIN       GPIO_Pin_11    /* PB11 → SDA 数据线 */
#define I2C_PORT          GPIOB          /* 使用 GPIOB 端口 */
#define I2C_RCC_CLOCK     RCC_APB2Periph_GPIOB  /* GPIOB 时钟 */

/* ========== 底 层 GPIO 操 作 宏 ========== */
/**
 * @name  Low-level bus operations
 * @note  Open-drain: SET releases the line (pull-up drives it high),
 *        CLR pulls it low; SDA_READ polls the input register.
 */
#define I2C_SCL_H()       GPIO_SetBits(I2C_PORT, I2C_SCL_PIN)
#define I2C_SCL_L()       GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN)
#define I2C_SDA_H()       GPIO_SetBits(I2C_PORT, I2C_SDA_PIN)
#define I2C_SDA_L()       GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN)

#define I2C_SDA_READ()    GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN)

/* ========== 对 外 接 口 函 数 ========== */

/**
 * @brief  Initialize I2C pins as open-drain outputs.
 * @note   Prefixed "Soft" to avoid clashing with the STM32 I2C peripheral.
 */
void SoftI2C_Init(void);

/** @brief Generate I2C start condition. */
void I2C_Start(void);
/** @brief Generate I2C stop condition. */
void I2C_Stop(void);

/** @brief Transmit one byte, MSB first.
 *  @param data: byte to send */
void I2C_SendByte(uint8_t data);
/** @brief Receive one byte, MSB first.
 *  @param ack: ACK to send after the byte (0 = ACK, 1 = NAK)
 *  @return received byte */
uint8_t I2C_ReadByte(uint8_t ack);

/** @brief Wait for slave ACK. @return 0 = ACK, 1 = NAK */
uint8_t I2C_WaitAck(void);

#endif /* __I2C_H */
