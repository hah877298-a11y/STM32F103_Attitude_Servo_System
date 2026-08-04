#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f10x.h"

/**
 * @file  Filter.h
 * @brief First-order complementary filter for attitude estimation
 *
 * Fuses gyro integration (short-term) with accel-derived angles
 * (long-term) to reject drift and vibration noise. Pitch/roll only.
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

void Filter_Init(ComplementaryFilter *f, float alpha, float dt);

/**
 * @brief  Compute pitch/roll from raw accelerometer data
 */
void Filter_AccelAngle(int16_t ax, int16_t ay, int16_t az, float *pitch, float *roll);

/**
 * @brief  Single-axis complementary filter update
 * @return Fused angle (deg)
 */
float Filter_Update(ComplementaryFilter *f, float gyro_rate, float accel_angle);

/**
 * @brief  Fuse pitch and roll in a single call
 */
void Filter_UpdateAttitude(ComplementaryFilter *pitch_f, ComplementaryFilter *roll_f,
                           float gx, float gy, float gz,
                           float acc_pitch, float acc_roll,
                           AttitudeAngle *out);

#endif /* __FILTER_H */
