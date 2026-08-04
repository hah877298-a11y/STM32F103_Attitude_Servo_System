#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"

/* ============================================================
 *                  SSD1306 OLED 显 示 驱 动 头 文 件
 * ============================================================
 *
 *  硬件规格:
 *    芯片:    SSD1306
 *    分辨率:  128 × 64 像素
 *    接口:    I2C (4 线: VCC, GND, SCL, SDA)
 *    地址:    0x3C (7位), 写地址 = 0x78
 *
 *  依赖: 需要先初始化 i2c.h 中的软件 I2C 驱动
 *
 *  坐标系:
 *    X 轴 (列): 0 在最左边, 127 在最右边
 *    Y 轴 (页): 0 在最顶部, 7 在最底部
 *    字体:      6×8 点阵 (宽 6 列 × 高 8 像素 = 占 1 页)
 *    每屏容量:  128÷6≈21 字符/行 × 8 行
 * ============================================================
 */

/* ========== OLED I2C 地 址 ========== */
#define OLED_ADDR       0x3C        /* SSD1306 的 7 位 I2C 地址 (最常见) */
#define OLED_ADDR_WRITE (OLED_ADDR << 1)    /* 写地址 = 0x78 */

/* ========== I2C 控 制 字 节 (区分命令和数据) ========== */
/*
 * SSD1306 每次 I2C 传输的第一个字节不是寄存器地址,
 * 而是一个"控制字节", 告诉芯片后续字节是命令还是数据:
 *
 *   Co = 0, D/C# = 0 → 控制字节 = 0x00 → 后续字节都是命令
 *   Co = 0, D/C# = 1 → 控制字节 = 0x40 → 后续字节都是数据 (写入 GDDRAM)
 *
 * 这是 SSD1306 I2C 协议与 MPU6050 最大的不同!
 * MPU6050 第一个数据字节是"寄存器地址",
 * SSD1306 第一个数据字节是"命令/数据标志".
 */
#define OLED_CTRL_CMD   0x00        /* 下一个字节是命令 */
#define OLED_CTRL_DATA  0x40        /* 下一个字节是数据 (写入显存) */

/* ========== 屏 幕 尺 寸 ========== */
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_PAGES      8           /* 64÷8=8 页 */

/* ========== 对 外 接 口 函 数 ========== */

void OLED_Init(void);               /* 初始化 OLED: 发一堆配置命令, 点亮屏幕 */
void OLED_Clear(void);              /* 清屏 (所有像素熄灭) */
void OLED_Fill(void);               /* 全屏填充 (所有像素点亮, 测试用) */

void OLED_ShowChar(uint8_t x, uint8_t y, char ch);         /* 在 (x列, y页) 显示一个字符 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str); /* 显示字符串 (自动换行) */
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len);  /* 显示有符号整数 */
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t intLen, uint8_t decLen); /* 显示浮点数 */

void OLED_SetCursor(uint8_t page, uint8_t col);  /* 设置光标位置 */

#endif /* __OLED_H */
