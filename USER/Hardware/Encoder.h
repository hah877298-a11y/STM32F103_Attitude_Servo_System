#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/**
 * @file    Encoder.h
 * @brief   Rotary encoder driver (EXTI).
 * @note    A = PB12 (EXTI, pulse), B = PB13 (EXTI, direction),
 *          C = GND, SW = PB14 (pull-up, low when pressed).
 *          Direction on A falling edge: B low -> CW, B high -> CCW.
 *          Used to adjust the PID setpoint (+/-5 deg per step).
 */

/* ---- GPIO macros ---- */
#define ENCODER_PORT        GPIOB
#define ENCODER_A_PIN       GPIO_Pin_12   /* A phase -> PB12 */
#define ENCODER_B_PIN       GPIO_Pin_13   /* B phase -> PB13 */
#define ENCODER_SW_PIN      GPIO_Pin_14   /* SW button -> PB14 */
#define ENCODER_RCC_CLOCK   RCC_APB2Periph_GPIOB

/* ---- public API ---- */

/** @brief Initialize encoder GPIO + EXTI interrupts. */
void Encoder_Init(void);

/** @brief Get accumulated pulse count. */
int16_t Encoder_GetCount(void);
/** @brief Reset accumulated pulse count. */
void Encoder_ResetCount(void);

/** @brief Encoder pulse handler; call from EXTI15_10_IRQHandler. */
void Encoder_OnInterrupt(void);

/** @brief Debounced SW read; @return 1 on a valid press. */
uint8_t Encoder_SW_Pressed(void);

#endif /* __ENCODER_H */
