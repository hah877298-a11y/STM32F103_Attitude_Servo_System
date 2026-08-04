#include "Encoder.h"
#include "SysTick.h"

/**
 * @file    Encoder.c
 * @brief   Rotary encoder driver using EXTI interrupts.
 * @note    A phase = PB12 (EXTI12, falling edge), B phase = PB13
 *          (EXTI13, read for direction); both share EXTI15_10_IRQn.
 *          Interrupt sampling avoids missed pulses under 10 ms polling.
 *          Debounce: 5 ms timestamp check (mechanical bounce 1..3 ms).
 */

/* software debounce interval (ms) */
#define ENCODER_DEBOUNCE_MS   5

/* volatile: updated in ISR, read in main loop */
static volatile int16_t encoder_count = 0;         /* accumulated pulses */
static volatile uint32_t last_interrupt_time = 0;  /* last ISR timestamp (debounce) */

/**
 * @brief  Initialize encoder GPIO (pull-up inputs) and EXTI interrupts.
 * @note   AFIO must be clocked to route pins to EXTI lines.
 */
void Encoder_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    EXTI_InitTypeDef   EXTI_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    /* GPIOB (APB2) + AFIO clocks (AFIO selects EXTI line sources) */
    RCC_APB2PeriphClockCmd(ENCODER_RCC_CLOCK | RCC_APB2Periph_AFIO, ENABLE);

    /* pull-up inputs: idle high, contact pulls low */
    GPIO_InitStructure.GPIO_Pin   = ENCODER_A_PIN | ENCODER_B_PIN | ENCODER_SW_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ENCODER_PORT, &GPIO_InitStructure);

    /* PB12 -> EXTI12, PB13 -> EXTI13 (shared EXTI15_10_IRQn) */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);

    /* falling edge: A high->low = one encoder step */
    EXTI_InitStructure.EXTI_Line    = EXTI_Line12 | EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* shared vector EXTI15_10, preemption priority 2 */
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Encoder ISR handler; call from EXTI15_10_IRQHandler.
 * @note   Direction from B level on A's falling edge: B = 0 -> CW,
 *         B = 1 -> CCW. Debounce by ISR timestamp. Keep it short.
 */
void Encoder_OnInterrupt(void)
{
    uint32_t now = SysTick_Get();

    /* debounce: ignore events closer than the debounce interval */
    if (now - last_interrupt_time < ENCODER_DEBOUNCE_MS)
    {
        last_interrupt_time = now;  /* still update: bounce arrives in bursts */
        return;
    }

    last_interrupt_time = now;

    if (GPIO_ReadInputDataBit(ENCODER_PORT, ENCODER_B_PIN) == Bit_RESET)
    {
        encoder_count++;            /* B low: A leads B -> CW */
    }
    else
    {
        encoder_count--;            /* B high: B leads A -> CCW */
    }
}

/* ============================================================
 *              获 取 / 清 零 编 码 器 计 数
 * ============================================================
 */
/**
 * @brief  Get the accumulated pulse count (CW positive, CCW negative).
 * @return current pulse count
 */
int16_t Encoder_GetCount(void)
{
    return encoder_count;
}

/**
 * @brief  Reset the accumulated pulse count to zero.
 * @note   Read-then-reset has a tiny race window vs. the ISR; at encoder
 *         rates (~tens of Hz) losing an occasional pulse is acceptable.
 */
void Encoder_ResetCount(void)
{
    encoder_count = 0;
}

/**
 * @brief  Debounced SW button read (pull-up: pressed = low).
 * @return 1 on a valid press (double-checked with delay), else 0
 * @note   Blocking debounce ~20 ms is acceptable in the 200 ms task;
 *         the release wait is timeout-guarded so a long press cannot
 *         stall the IWDG feed.
 */
uint8_t Encoder_SW_Pressed(void)
{
    if (GPIO_ReadInputDataBit(ENCODER_PORT, ENCODER_SW_PIN) == Bit_RESET)
    {
        /* ~20 ms debounce */
        {
            volatile uint32_t i;
            for (i = 0; i < 200000; i++) __NOP();
        }

        /* still low: confirmed press */
        if (GPIO_ReadInputDataBit(ENCODER_PORT, ENCODER_SW_PIN) == Bit_RESET)
        {
            /* wait for release, max ~500 ms */
            volatile uint32_t timeout = 5000000;
            while (GPIO_ReadInputDataBit(ENCODER_PORT, ENCODER_SW_PIN) == Bit_RESET
                   && --timeout);
            return 1;
        }
    }
    return 0;
}
