#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"

/* ============================================================
 *             SysTick 系 统 节 拍 定 时 器 头 文 件
 * ============================================================
 *
 *  功能:
 *    为整个工程提供毫秒级时钟基准.
 *    SysTick 是 Cortex-M3 内核自带的 24 位定时器,
 *    不需要占用 STM32 的外设定时器 (TIM2/3/4).
 *
 *  中断周期: 1ms (1000Hz)
 *  计数范围: 0 → 2^32-1 毫秒 (约 49.7 天才会溢出一次,
 *            但因为用无符号数相减, 即使溢出也不影响计时精度)
 *
 *  使用方法:
 *    1. main() 中调用 SysTick_Init() 启动
 *    2. 任意位置调用 SysTick_Get() 获取当前毫秒数
 *    3. 用 if (now - last >= interval) 实现非阻塞定时:
 *
 *       uint32_t last = 0;
 *       while (1) {
 *           uint32_t now = SysTick_Get();
 *           if (now - last >= 500) {   // 每 500ms 执行一次
 *               last = now;
 *               // 做点什么...
 *           }
 *       }
 *
 *  与 delay_ms() 的本质区别:
 *    delay_ms(500) → CPU 空转 500ms, 期间什么都不做
 *    if (now - last >= 500) → CPU 只花几微秒检查时间,
 *                             不满足就立刻执行其他任务,
 *                             满足了才进入——这 500ms 里 CPU 没有白等!
 * ============================================================
 */

/* ========== 对 外 接 口 ========== */

void SysTick_Init(void);         /* 初始化 SysTick, 开启 1ms 中断 */
uint32_t SysTick_Get(void);      /* 获取从启动到现在的毫秒数 */
void SysTick_IncTick(void);      /* 在 SysTick_Handler 中调用, tick+1 (内部使用) */

#endif /* __SYSTICK_H */
