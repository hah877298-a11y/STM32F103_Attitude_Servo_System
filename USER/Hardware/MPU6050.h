#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

/* ============================================================
 *                     MPU6050 驱 动 头 文 件
 * ============================================================
 *
 *  硬件连接:
 *    VCC  → 3.3V
 *    GND  → GND
 *    SCL  → PB10
 *    SDA  → PB11
 *    AD0  → GND (7位从机地址 = 0x68)
 *
 *  依赖: 需要先初始化 i2c.h 中的软件 I2C 驱动
 * ============================================================
 */

/* ========== MPU6050 I2C 从 机 地 址 ========== */
/*
 * AD0 引脚接 GND → 7 位地址 = 0x68
 * AD0 引脚接 VCC → 7 位地址 = 0x69
 *
 * 在 I2C 总线上, 第一个字节 = (7位地址 << 1) | 方向位
 *   方向位: 0 = 主机写入从机, 1 = 主机读取从机
 */
#define MPU6050_ADDR            0x68   /* 7位地址 (AD0=GND) */
#define MPU6050_ADDR_WRITE      (MPU6050_ADDR << 1)       /* 0xD0 */
#define MPU6050_ADDR_READ       (MPU6050_ADDR << 1 | 1)   /* 0xD1 */

/* ========== MPU6050 内 部 寄 存 器 地 址 ========== */
/* 只列出本实验中用到的寄存器, 完整列表见 MPU6050 数据手册 */

/* ---- 配置寄存器 ---- */
#define MPU6050_REG_SMPLRT_DIV   0x19   /* 采样率分频器 (默认 8kHz 陀螺输出) */
#define MPU6050_REG_CONFIG       0x1A   /* 数字低通滤波器 (DLPF) 配置 */
#define MPU6050_REG_GYRO_CONFIG  0x1B   /* 陀螺仪量程配置 */
#define MPU6050_REG_ACCEL_CONFIG 0x1C   /* 加速度计量程配置 */

/* ---- 数据寄存器 (从 0x3B 开始, 连续 14 字节) ---- */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B   /* 加速度 X 轴高字节 (从这里读 14 字节拿到全部数据) */

/* ---- 控制和状态寄存器 ---- */
#define MPU6050_REG_PWR_MGMT_1   0x6B   /* 电源管理 1: 控制睡眠/唤醒/时钟源 */
#define MPU6050_REG_WHO_AM_I     0x75   /* "我是谁" 寄存器 (读出来永远是 0x68, 用于验证 I2C 通信) */

/* ========== 加 速 度 和 陀 螺 仪 的 量 程 换 算 因 子 ========== */
/*
 * 传感器输出的是 16 位有符号原始值 (-32768 ~ +32767),
 * 需要除以换算因子才能得到实际的物理单位 (g 或 °/s).
 *
 * 量程越大, 能测量的范围越广, 但精度越低.
 * 我们的配置 (GYRO=±2000°/s, ACCEL=±16g):
 */
#define MPU6050_GYRO_SCALE       16.4f    /* 陀螺仪 ±2000°/s: 1 LSB = 2000/32768 ≈ 0.06098 °/s
                                             换算因子 = 32768/2000 = 16.384 ≈ 16.4 LSB/(°/s) */
#define MPU6050_ACCEL_SCALE      2048.0f  /* 加速度 ±16g: 1 LSB = 16/32768 ≈ 0.000488 g
                                             换算因子 = 32768/16 = 2048 LSB/g */

/* ========== 数 据 结 构 ========== */
/**
 * @brief  MPU6050 原始传感器数据 (从寄存器直接读出来的值)
 *
 *  每个轴的值是 16 位有符号整数, 范围 -32768 ~ +32767.
 *  需要除以量程换算因子才能得到实际的物理值.
 */
typedef struct
{
    int16_t Accel_X;    /* 加速度 X 轴原始值 */
    int16_t Accel_Y;    /* 加速度 Y 轴原始值 */
    int16_t Accel_Z;    /* 加速度 Z 轴原始值 */
    int16_t Temp;       /* 温度原始值 */
    int16_t Gyro_X;     /* 陀螺仪 X 轴原始值 */
    int16_t Gyro_Y;     /* 陀螺仪 Y 轴原始值 */
    int16_t Gyro_Z;     /* 陀螺仪 Z 轴原始值 */
} MPU6050_RawData;

/**
 * @brief  MPU6050 转换后的物理数据
 *
 *  Accel 单位: g (1g = 地球重力加速度 ≈ 9.8 m/s²)
 *  Gyro  单位: °/s (度/秒)
 *  Temp  单位: °C (摄氏度)
 */
typedef struct
{
    float Accel_X_g;    /* 加速度 X 轴 (g) */
    float Accel_Y_g;    /* 加速度 Y 轴 (g) */
    float Accel_Z_g;    /* 加速度 Z 轴 (g) */
    float Gyro_X_dps;   /* 陀螺仪 X 轴 (°/s) */
    float Gyro_Y_dps;   /* 陀螺仪 Y 轴 (°/s) */
    float Gyro_Z_dps;   /* 陀螺仪 Z 轴 (°/s) */
    float Temp_C;       /* 温度 (°C) */
} MPU6050_PhyData;

/* ========== 对 外 接 口 函 数 ========== */

void MPU6050_Init(void);          /* 初始化 MPU6050: 唤醒 + 配置量程和采样率 */

uint8_t MPU6050_ReadID(void);     /* 读取 WHO_AM_I 寄存器, 验证 I2C 通信是否正常 */

void MPU6050_ReadRawData(MPU6050_RawData *data);    /* 读取全部 7 组原始数据 */
void MPU6050_ConvertToPhy(const MPU6050_RawData *raw, MPU6050_PhyData *phy); /* 原始值 → 物理值 */

/* ---- 底层寄存器读写 (也可供其他模块调用) ---- */
void MPU6050_WriteReg(uint8_t reg, uint8_t value);              /* 写一个寄存器 */
void MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);  /* 连续读多个寄存器 */

#endif /* __MPU6050_H */
