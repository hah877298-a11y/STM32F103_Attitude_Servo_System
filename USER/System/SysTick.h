#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"

/**
 * @file    SysTick.h
 * @brief   SysTick millisecond tick driver interface.
 *          Provides a 1ms non-blocking time base using the Cortex-M3
 *          core timer; no TIM peripheral is occupied.
 */

void SysTick_Init(void);         /* 1 ms interrupt */
uint32_t SysTick_Get(void);      /* elapsed ms since init */
void SysTick_IncTick(void);      /* ISR only */
void SysTick_DelayMs(uint32_t ms); /* blocking; init only, not for main loop */

#endif /* __SYSTICK_H */
