#ifndef __I2C_H
#define __I2C_H

#include "stm32f10x.h"

/* ============================================================
 *                   软 件 模 拟 I2C 头 文 件
 * ============================================================
 *
 *  引脚接线:
 *    PB10  →  SCL (时钟线, 接 4.7kΩ 上拉电阻到 3.3V)
 *    PB11  →  SDA (数据线, 接 4.7kΩ 上拉电阻到 3.3V)
 *
 *  说明: 本驱动使用 GPIO 开漏输出模拟 I2C 总线时序,
 *        可同时挂载多个从机设备 (如 MPU6050 + OLED),
 *        只要它们的 7 位从机地址不冲突即可.
 * ============================================================
 */

/* ========== 引脚宏定义 (方便修改) ========== */
#define I2C_SCL_PIN       GPIO_Pin_10    /* PB10 → SCL 时钟线 */
#define I2C_SDA_PIN       GPIO_Pin_11    /* PB11 → SDA 数据线 */
#define I2C_PORT          GPIOB          /* 使用 GPIOB 端口 */
#define I2C_RCC_CLOCK     RCC_APB2Periph_GPIOB  /* GPIOB 时钟 */

/* ========== 底 层 GPIO 操 作 宏 ========== */
/*
 * 这些宏直接操作 GPIO 寄存器, 比调用库函数更快.
 * 软件 I2C 对时序敏感, 用宏可以减少函数调用开销.
 *
 * 理解要点:
 *   - 引脚配置为"开漏输出", SET(SDA=1) 实际上是让 NMOS 管截止,
 *     引脚变为高阻态, 由上拉电阻拉到高电平 → 等同于"释放总线"
 *   - CLR(SDA=0) 让 NMOS 管导通, 引脚直连 GND → "拉低总线"
 *   - SDA_READ 读取的是 GPIO 输入数据寄存器 (IDR) 的实际电平,
 *     不受 ODR 影响, 所以能正确读到从机拉低的信号
 */
#define I2C_SCL_H()       GPIO_SetBits(I2C_PORT, I2C_SCL_PIN)
#define I2C_SCL_L()       GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN)
#define I2C_SDA_H()       GPIO_SetBits(I2C_PORT, I2C_SDA_PIN)
#define I2C_SDA_L()       GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN)

/* 读取 SDA 引脚的实际电平 (用于检测从机应答/读取数据) */
#define I2C_SDA_READ()    GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN)

/* ========== 对 外 接 口 函 数 ========== */

void SoftI2C_Init(void);       /* GPIO 初始化, 配置开漏输出 (为避免与硬件I2C库冲突, 加 Soft 前缀) */

void I2C_Start(void);          /* 产生 I2C 起始信号 */
void I2C_Stop(void);           /* 产生 I2C 停止信号 */

void I2C_SendByte(uint8_t data);   /* 发送 1 个字节 (MSB 在前) */
uint8_t I2C_ReadByte(uint8_t ack); /* 读取 1 个字节, 参数决定回复 ACK/NAK */

uint8_t I2C_WaitAck(void);     /* 等待从机应答, 返回 0=ACK / 1=NAK */

#endif /* __I2C_H */
