#ifndef __TASK_SCHEDULER_H
#define __TASK_SCHEDULER_H

/**
 * @file    task_scheduler.h
 * @brief   Non-blocking time-sliced task scheduler interface.
 *
 * Five cooperative tasks run at fixed intervals (10/20/50/200/1000 ms),
 * polled from main() via Scheduler_Run(); no preemption.
 */

#include "stm32f10x.h"

/* ---- public interface ---- */

/**
 * @brief  One-time initialization of all modules and the scheduler.
 * @note   Call once from main() before entering the run loop.
 *          - SysTick, USART+DMA, IWDG
 *          - I2C bus, OLED, MPU6050
 *          - Servo PWM, Encoder, ADC
 *          - Complementary filter, PID
 */
void Scheduler_Init(void);

/**
 * @brief  Execute one iteration of the cooperative task loop.
 * @note   Call continuously from main(); checks all five deadlines
 *         and dispatches any that are due.
 */
void Scheduler_Run(void);

#endif /* __TASK_SCHEDULER_H */
