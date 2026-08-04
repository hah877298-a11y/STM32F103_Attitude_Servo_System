/**
 * @file    SysTick.c
 * @brief   SysTick millisecond tick driver providing the non-blocking
 *          time base for task scheduling.
 */
#include "SysTick.h"

/** Global millisecond counter, incremented in the SysTick ISR. */
static volatile uint32_t sysTickCounter = 0;

/**
 * @brief  Initialize SysTick to generate an interrupt every 1ms.
 * @note   Reload value is derived from SystemCoreClock, so the
 *         driver adapts to any system clock frequency.
 */
void SysTick_Init(void)
{
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Config failed - halt for debugging */
        while (1);
    }
}

/**
 * @brief  Get the current millisecond tick count.
 * @return Milliseconds elapsed since SysTick_Init(); wraps after
 *         ~49.7 days (unsigned subtraction stays correct).
 */
uint32_t SysTick_Get(void)
{
    return sysTickCounter;
}

/**
 * @brief  Increment the millisecond counter.
 * @note   Called from SysTick_Handler only; keeps the ISR minimal
 *         and hides the counter from other modules.
 */
void SysTick_IncTick(void)
{
    sysTickCounter++;
}
