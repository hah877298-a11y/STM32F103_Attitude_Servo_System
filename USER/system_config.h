#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

/**
 * @file    system_config.h
 * @brief   System-wide tuning parameters and scheduler constants.
 *
 * Centralizes parameters shared across modules (task periods, algorithm
 * defaults, calibration values). Pin macros stay in their respective
 * driver headers (i2c.h, Servo.h, Encoder.h, etc.).
 */

#include "stm32f10x.h"

/* ================================================================
 *  1.  System clock reference
 * ================================================================ */
#define SYSTEM_CLOCK_HZ         72000000UL      /* HSE 8 MHz x 9 PLL = 72 MHz */

/* ================================================================
 *  2.  Task scheduling periods (ms)
 * ================================================================ */
#define TASK_PERIOD_10MS        10
#define TASK_PERIOD_20MS        20
#define TASK_PERIOD_50MS        50
#define TASK_PERIOD_200MS       200
#define TASK_PERIOD_1000MS      1000

/* ================================================================
 *  3.  IWDG independent watchdog
 * ================================================================ */
#define IWDG_PRESCALER          IWDG_Prescaler_64   /* LSI 40 kHz / 64 = 625 Hz */
#define IWDG_RELOAD             1250                /* 1250 / 625 Hz = 2.0 s timeout */

/* ================================================================
 *  4.  ADC analog sensor reference
 * ================================================================ */
#define ADC_VREF_MV             3300            /* reference voltage (mV) */
#define ADC_RESOLUTION          4096            /* 12-bit ADC */

/* ================================================================
 *  5.  Complementary filter defaults
 * ================================================================ */
#define FILTER_DEFAULT_ALPHA    0.98f           /* weight of gyro integration */
#define FILTER_DT_S             0.010f          /* sampling interval (s) */

/* ================================================================
 *  6.  PID defaults
 * ================================================================ */
#define PID_DEFAULT_KP          2.0f
#define PID_DEFAULT_KI          0.05f
#define PID_DEFAULT_KD          0.5f
#define PID_INTEGRAL_LIMIT      20.0f           /* anti-windup clamp */
#define PID_OUTPUT_MIN          0.0f            /* servo angle min (deg) */
#define PID_OUTPUT_MAX          180.0f          /* servo angle max (deg) */

/* ================================================================
 *  7.  Encoder setpoint and tuning
 * ================================================================ */
#define ENCODER_STEP_DEG        0.5f            /* degrees per encoder pulse */
#define SETPOINT_MIN_DEG        -45.0f
#define SETPOINT_MAX_DEG        45.0f

/* PID tuning step sizes (per encoder detent) */
#define TUNE_STEP_KP            0.1f
#define TUNE_STEP_KI            0.01f
#define TUNE_STEP_KD            0.05f

/* ================================================================
 *  8.  VOFA+ serial report
 * ================================================================ */
#define VOFA_MAX_CHANNELS       8               /* FireWater CSV channels */

/* Enable periodic VOFA+ (FireWater CSV) frame transmission */
#define VOFA_ENABLE

#endif /* __SYSTEM_CONFIG_H */
