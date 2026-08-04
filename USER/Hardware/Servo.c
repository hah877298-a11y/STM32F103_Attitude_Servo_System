#include "Servo.h"

/**
 * @file    Servo.c
 * @brief   SG90 servo driver (PWM on TIM2_CH1 / PA0).
 * @note    Period 20 ms (50 Hz); pulse 0.5..2.5 ms maps linearly to
 *          0..180 deg: pulse_us = 500 + angle * 2000 / 180.
 *          TIM2 clock 72 MHz (APB1 x2); PSC 71 -> 1 MHz (1 us/tick),
 *          ARR 19999 -> 20 ms period, CCR = pulse width in us.
 *          Servo requires external 5 V supply (stall ~750 mA) - never
 *          power it from the 3.3 V rail.
 */

/**
 * @brief  Initialize TIM2_CH1 (PA0) PWM output for the servo.
 * @note   Starts with 1.5 ms pulse -> servo parks at 90 deg.
 */
void Servo_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* enable GPIOA (APB2) and TIM2 (APB1) clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* PA0: TIM2_CH1 alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 72 MHz / 72 = 1 MHz (1 us/tick); 20000 ticks = 20 ms period */
    TIM_TimeBaseStructure.TIM_Period        = 20000 - 1;   /* ARR: 20 ms */
    TIM_TimeBaseStructure.TIM_Prescaler     = 72 - 1;      /* PSC: 1 MHz */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* PWM1: output high while CNT < CCR (CCR = pulse width) */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 1500;  /* 1.5 ms -> 90 deg */
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);

    /* preload CCR: new values take effect at the next period start */
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);               /* start PWM output */
}

/**
 * @brief  Set servo angle, linearly mapped to pulse width.
 * @param  angle: target angle, clamped to 0..180 deg
 * @note   Mapping: pulse = 500 + angle * 2000 / 180 (us).
 */
void Servo_SetAngle(int16_t angle)
{
    uint16_t pulse;

    if (angle < SERVO_MIN_ANGLE)
        angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;

    /* multiply first, then divide, to keep precision */
    pulse = (uint16_t)(SERVO_MIN_PULSE +
             (uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180);

    TIM_SetCompare1(TIM2, pulse);   /* applied at next period (preload) */
}

/**
 * @brief  Set PWM pulse width directly in microseconds (500..2500).
 * @param  pulse_us: pulse width (us), clamped to 500..2500
 * @note   Bypasses angle mapping; for calibration and PID pulse output.
 */
void Servo_SetPulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE)
        pulse_us = SERVO_MIN_PULSE;
    if (pulse_us > SERVO_MAX_PULSE)
        pulse_us = SERVO_MAX_PULSE;

    TIM_SetCompare1(TIM2, pulse_us);
}
