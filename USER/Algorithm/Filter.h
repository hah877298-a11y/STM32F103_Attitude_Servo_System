#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f10x.h"

/**
 * @file  Filter.h
 * @brief Complementary filter for attitude estimation plus general
 *        digital filters (moving-average, limit, median).
 */

/**
 * @brief  Complementary filter instance (one per axis)
 */
typedef struct
{
    float angle;        /* fused angle (deg) */
    float bias;         /* gyro bias (deg/s), reserved */
    float alpha;        /* filter coefficient (0~1, typical 0.98) */
    float dt;           /* sampling interval (s) */
} ComplementaryFilter;

/**
 * @brief  Fused attitude angles
 */
typedef struct
{
    float pitch;    /* pitch angle (deg) */
    float roll;     /* roll angle (deg) */
    float yaw;      /* yaw angle (deg), not tracked */
} AttitudeAngle;

/* ================================================================
 *  Complementary filter (attitude fusion)
 * ================================================================ */

void Filter_Init(ComplementaryFilter *f, float alpha, float dt);
void Filter_AccelAngle(int16_t ax, int16_t ay, int16_t az, float *pitch, float *roll);
float Filter_Update(ComplementaryFilter *f, float gyro_rate, float accel_angle);
void Filter_UpdateAttitude(ComplementaryFilter *pitch_f, ComplementaryFilter *roll_f,
                           float gx, float gy, float gz,
                           float acc_pitch, float acc_roll,
                           AttitudeAngle *out);

/* ================================================================
 *  Moving-average filter (general noise suppression)
 * ================================================================ */

#define MA_WINDOW  8   /* buffer size, must be power of 2 for fast modulo */

typedef struct
{
    float   buf[MA_WINDOW];
    float   sum;
    uint8_t idx;
    uint8_t full;       /* 0 until the first full rotation */
} MovAvg_t;

void  MovAvg_Init(MovAvg_t *f);
float MovAvg_Update(MovAvg_t *f, float in);

/* ================================================================
 *  Limit filter (spike / glitch rejection)
 * ================================================================ */

float LimitFilter(float in, float prev, float threshold);

/* ================================================================
 *  Median filter (isolated outlier removal)
 * ================================================================ */

#define MEDIAN_N  5

float MedianFilter(float in);

/* ================================================================
 *  ADC floating-pin detector
 * ================================================================ */

typedef enum
{
    CH_OK = 0,
    CH_FLOATING,        /* pin floating, level bouncing */
    CH_STUCK_LOW,       /* short to GND, always ~0 */
    CH_STUCK_HIGH       /* short to VCC, always ~3.3 V */
} ChannelStatus_t;

ChannelStatus_t ADC_CheckChannel(MovAvg_t *ma, float raw);

#endif /* __FILTER_H */
