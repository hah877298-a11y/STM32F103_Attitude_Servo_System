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

/**
 * @brief  Blocking delay in milliseconds, based on the 1 ms tick counter.
 * @param  ms: delay length, must be > 0
 * @note   Init sequences only; never call from the main task loop
 *         (would break the non-blocking architecture).
 */
void SysTick_DelayMs(uint32_t ms)
{
    uint32_t target = sysTickCounter + ms;
    while (sysTickCounter < target)
    {
        /* spin; ISR advances sysTickCounter every 1 ms */
    }
}
