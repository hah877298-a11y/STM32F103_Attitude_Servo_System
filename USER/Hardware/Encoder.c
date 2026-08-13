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
 *         B = 1 -> CCW. Debounce via ISR timestamp.
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

/* ---- count access ---- */
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

/* ---- SW non-blocking debounce state machine ---- */
/* Polled every 200 ms from the main loop.
 *   IDLE         -> pin low          -> DEBOUNCE (record timestamp)
 *   DEBOUNCE     -> 20 ms elapsed, still low -> WAIT_RELEASE
 *                 -> 20 ms elapsed, high     -> IDLE (glitch)
 *   WAIT_RELEASE -> pin high         -> IDLE, return 1 (valid press)
 *                 -> 500 ms timeout  -> IDLE (long press, ignored)
 * All timing via SysTick timestamps; never blocks. */

#define SW_DEBOUNCE_MS     20      /* debounce interval (ms) */
#define SW_HOLD_TIMEOUT_MS  500     /* long-press timeout (ms) */

typedef enum {
    SW_IDLE = 0,
    SW_DEBOUNCE,
    SW_WAIT_RELEASE
} EncoderSW_State;

static EncoderSW_State sw_state = SW_IDLE;
static uint32_t sw_timestamp = 0;

/**
 * @brief  Non-blocking debounced SW button read (pull-up: pressed = low).
 * @return 1 on a valid short press (press + release), else 0
 * @note   Poll at >= 50 Hz. State machine advances on each call;
 *         never blocks the caller.
 */
uint8_t Encoder_SW_Pressed(void)
{
    uint32_t now = SysTick_Get();
    uint8_t  pin = GPIO_ReadInputDataBit(ENCODER_PORT, ENCODER_SW_PIN);

    switch (sw_state)
    {
    case SW_IDLE:
        if (pin == Bit_RESET)           /* pressed */
        {
            sw_state     = SW_DEBOUNCE;
            sw_timestamp = now;
        }
        break;

    case SW_DEBOUNCE:
        if (now - sw_timestamp < SW_DEBOUNCE_MS)
        {
            /* still inside debounce window; wait */
            break;
        }

        if (pin == Bit_RESET)           /* confirmed press */
        {
            sw_state     = SW_WAIT_RELEASE;
            sw_timestamp = now;
        }
        else                            /* glitch: discard */
        {
            sw_state = SW_IDLE;
        }
        break;

    case SW_WAIT_RELEASE:
        if (pin != Bit_RESET)           /* released -> valid short press */
        {
            sw_state = SW_IDLE;
            return 1;
        }

        /* long-press guard: ignore holds > 500 ms */
        if (now - sw_timestamp > SW_HOLD_TIMEOUT_MS)
        {
            sw_state = SW_IDLE;
        }
        break;
    }

    return 0;
}
