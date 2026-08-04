#include "Filter.h"
#include <math.h>

/**
 * @file  Filter.c
 * @brief First-order complementary filter (accel + gyro fusion)
 */

/**
 * @brief  Initialize complementary filter
 * @param  f      Filter instance
 * @param  alpha  Filter coefficient, typical 0.96~0.99
 * @param  dt     Sampling interval in seconds (e.g. 0.01 for 10ms)
 */
void Filter_Init(ComplementaryFilter *f, float alpha, float dt)
{
    f->angle = 0.0f;
    f->bias  = 0.0f;
    f->alpha = alpha;
    f->dt    = dt;
}

/**
 * @brief  Compute pitch/roll angles from raw accelerometer data
 * @param  ax,ay,az  Raw accelerometer readings (LSB)
 * @param  pitch     Output pitch angle (deg)
 * @param  roll      Output roll angle (deg)
 *
 * @note   Raw LSB values are used directly; the scale factor
 *         cancels out inside atan2f.
 */
void Filter_AccelAngle(int16_t ax, int16_t ay, int16_t az,
                       float *pitch, float *roll)
{
    *pitch = atan2f((float)ax,
                    sqrtf((float)ay * ay + (float)az * az))
             * 57.29578f;   /* rad -> deg (180/PI) */

    *roll  = atan2f((float)ay,
                    sqrtf((float)ax * ax + (float)az * az))
             * 57.29578f;
}

/**
 * @brief  Single-axis complementary filter update
 * @param  f            Filter instance
 * @param  gyro_rate    Gyro angular rate (deg/s)
 * @param  accel_angle  Accel-derived angle (deg)
 * @return Fused angle (deg)
 *
 * @note   angle = alpha*(angle + gyro*dt) + (1-alpha)*accel_angle
 */
float Filter_Update(ComplementaryFilter *f, float gyro_rate, float accel_angle)
{
    f->angle = f->alpha * (f->angle + gyro_rate * f->dt)
             + (1.0f - f->alpha) * accel_angle;

    return f->angle;
}

/**
 * @brief  Fuse pitch and roll attitude in a single call
 * @param  pitch_f,roll_f  Filter instances for pitch/roll
 * @param  gx,gy,gz        Gyro angular rates (deg/s)
 * @param  acc_pitch       Accel-derived pitch (deg)
 * @param  acc_roll        Accel-derived roll (deg)
 * @param  out             Output fused attitude
 */
void Filter_UpdateAttitude(ComplementaryFilter *pitch_f, ComplementaryFilter *roll_f,
                           float gx, float gy, float gz,
                           float acc_pitch, float acc_roll,
                           AttitudeAngle *out)
{
    /* axis mapping: pitch <- gyro_y, roll <- gyro_x */
    out->pitch = Filter_Update(pitch_f, gy, acc_pitch);
    out->roll  = Filter_Update(roll_f,  gx, acc_roll);

    out->yaw = 0.0f;  /* yaw not tracked */

    (void)gz;         /* gz unused: yaw intentionally ignored */
}
