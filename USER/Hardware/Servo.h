#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

/**
 * @file    Servo.h
 * @brief   SG90 servo driver (PWM on TIM2_CH1 / PA0).
 * @note    Wiring: signal PA0, power from external 5 V (shared GND).
 *          50 Hz PWM, pulse 0.5..2.5 ms -> 0..180 deg.
 *          Stall current ~750 mA: do NOT power from the 3.3 V rail or
 *          the USB port may brown out and reset the MCU.
 */

/* ---- angle range ---- */
#define SERVO_MIN_ANGLE     0       /* min angle (deg) */
#define SERVO_MAX_ANGLE     180     /* max angle (deg) */

/* ---- pulse range (us) ---- */
#define SERVO_MIN_PULSE     500     /* 0.5 ms -> 0 deg */
#define SERVO_MAX_PULSE     2500    /* 2.5 ms -> 180 deg */

/* ---- public API ---- */

/** @brief Initialize TIM2_CH1 PWM output (servo parks at 90 deg). */
void Servo_Init(void);
/** @brief Set servo angle, 0..180 deg (clamped). */
void Servo_SetAngle(int16_t angle);
/** @brief Set pulse width directly in us, 500..2500 (clamped). */
void Servo_SetPulse(uint16_t pulse_us);

#endif /* __SERVO_H */
