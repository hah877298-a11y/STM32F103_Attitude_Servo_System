#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"

/**
 * @file    SysTick.h
 * @brief   SysTick millisecond tick driver interface.
 *          Provides a 1ms non-blocking time base using the Cortex-M3
 *          core timer; no TIM peripheral is occupied.
 */

void SysTick_Init(void);         /* Init SysTick, 1ms interrupt */
uint32_t SysTick_Get(void);      /* Get elapsed milliseconds */
void SysTick_IncTick(void);      /* Increment tick counter (ISR internal) */

#endif /* __SYSTICK_H */
