#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/* ============================================================
 *            旋 转 编 码 器 驱 动 头 文 件
 * ============================================================
 *
 *  硬件连接:
 *    编码器 A 相 → PB12  (EXTI, 中断触发)
 *    编码器 B 相 → PB13  (EXTI, 判断方向)
 *    编码器 C 端 (公共端) → GND
 *    编码器 SW  (按键)   → PB14  (上拉输入, 按下为低电平)
 *
 *  旋转编码器原理 (以 EC11 为例):
 *    编码器内部有 A/B 两路开关, 旋转时交替通断.
 *    顺时针旋转: A 比 B 先闭合 → A 下降沿时 B=0
 *    逆时针旋转: B 比 A 先闭合 → A 下降沿时 B=1
 *
 *    时序图 (顺时针):
 *      A: ──┐     ┌──┐     ┌──
 *          └─────┘  └─────┘
 *      B:   ┌──┐     ┌──┐
 *      ────┘  └─────┘  └───
 *          ↑ A↓时 B=0 → CW
 *
 *    时序图 (逆时针):
 *      A:   ┌──┐     ┌──┐
 *      ────┘  └─────┘  └───
 *      B: ──┐     ┌──┐     ┌──
 *          └─────┘  └─────┘
 *          ↑ A↓时 B=1 → CCW
 *
 *  消抖策略:
 *    机械编码器在切换瞬间会有抖动 (几毫秒),
 *    导致一次旋转产生多个脉冲.
 *    软件消抖: 在中断中简单延时或记录上次中断时间,
 *    间隔太短 (< 5ms) 的触发视为抖动, 忽略.
 *
 *  在本工程中的作用:
 *    旋转编码器用于手动调节 PID 的目标角度 (setpoint).
 *    顺时针 → 目标角度 +5°
 *    逆时针 → 目标角度 -5°
 *    按下 SW (可选) → 复位到 0°
 * ============================================================
 */

/* ========== 编 码 器 GPIO 宏 定 义 ========== */
#define ENCODER_PORT        GPIOB
#define ENCODER_A_PIN       GPIO_Pin_12   /* A 相 → PB12 */
#define ENCODER_B_PIN       GPIO_Pin_13   /* B 相 → PB13 */
#define ENCODER_SW_PIN      GPIO_Pin_14   /* SW 按键 → PB14 */
#define ENCODER_RCC_CLOCK   RCC_APB2Periph_GPIOB

/* ========== 对 外 接 口 ========== */

void Encoder_Init(void);              /* 初始化编码器 GPIO + EXTI 中断 */

int16_t Encoder_GetCount(void);       /* 获取编码器累计脉冲数 */
void Encoder_ResetCount(void);        /* 清零累计脉冲数 */

/* 在 EXTI 中断服务函数中调用, 通知编码器有脉冲 */
void Encoder_OnInterrupt(void);

/* SW 按键: 读取当前电平 (0=按下, 1=释放), 带简单消抖 */
uint8_t Encoder_SW_Pressed(void);

#endif /* __ENCODER_H */
