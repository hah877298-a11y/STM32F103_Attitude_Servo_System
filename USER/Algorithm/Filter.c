#include "Filter.h"
#include <math.h>
#include <string.h>

/**
 * @file  Filter.c
 * @brief First-order complementary filter (accel + gyro fusion)
 *        + moving-average, limit, median filters for general use.
 */

/* ================================================================
 *  Complementary filter
 * ================================================================ */

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
 * @note   angle = alpha*(angle + gyro*dt) + (1-alpha)*accel_angle;
 *         time constant tau = alpha*dt/(1-alpha) ~ 0.49 s at
 *         alpha = 0.98, dt = 10 ms.
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

/* ================================================================
 *  Moving-average filter
 * ================================================================ */

/**
 * @brief  Initialize a moving-average filter instance.
 * @param  f  Filter instance to initialise
 */
void MovAvg_Init(MovAvg_t *f)
{
    memset(f->buf, 0, sizeof(f->buf));
    f->sum  = 0.0f;
    f->idx  = 0;
    f->full = 0;
}

/**
 * @brief  Push a new sample and return the window average.
 * @param  f   Filter instance (state updated in-place)
 * @param  in  New raw sample
 * @return Smoothed value (average of the last N samples)
 *
 * @note   Uses a circular buffer; sum is maintained incrementally
 *         so each call is O(1).  MA_WINDOW is power-of-2 so modulo
 *         compiles to a bitwise AND.
 */
float MovAvg_Update(MovAvg_t *f, float in)
{
    f->sum -= f->buf[f->idx];          /* evict oldest sample from sum */
    f->sum += in;                      /* fold in newest sample */
    f->buf[f->idx] = in;               /* overwrite buffer slot */
    f->idx = (f->idx + 1) & (MA_WINDOW - 1);

    if (!f->full)
    {
        if (f->idx == 0) f->full = 1;  /* wrapped once -> window full */
        return f->sum / (float)(f->idx == 0 ? MA_WINDOW : f->idx);
    }

    return f->sum * (1.0f / (float)MA_WINDOW);  /* multiply faster than divide */
}

/* ================================================================
 *  Limit filter (rate-of-change clamp)
 * ================================================================ */

/**
 * @brief  Clamp the step size between consecutive samples.
 * @param  in         New raw sample
 * @param  prev       Previous output value
 * @param  threshold  Max allowed change per sample step
 * @return Filtered value, limited to prev ± threshold
 *
 * @note   Zero-delay single-sample filter.  Useful for rejecting
 *         glitches that are much faster than the physical signal.
 */
float LimitFilter(float in, float prev, float threshold)
{
    float diff = in - prev;

    if      (diff >  threshold) return prev + threshold;
    else if (diff < -threshold) return prev - threshold;

    return in;  /* change within bounds */
}

/* ================================================================
 *  Median filter
 * ================================================================ */

/**
 * @brief  Return the median of the last MEDIAN_N samples.
 * @param  in  New raw sample
 * @return Median value of the sliding window
 *
 * @note   Uses a static ring buffer (single instance only). Removes
 *         isolated spike noise (ESD, loose connectors).
 */
float MedianFilter(float in)
{
    static float buf[MEDIAN_N];
    static uint8_t idx = 0;
    float tmp[MEDIAN_N];
    uint8_t i, j;

    buf[idx] = in;
    idx = (idx + 1) % MEDIAN_N;

    /* copy, then bubble-sort (N=5 -> 10 comparisons max) */
    for (i = 0; i < MEDIAN_N; i++) tmp[i] = buf[i];

    for (i = 0; i < MEDIAN_N - 1; i++)
    {
        for (j = 0; j < MEDIAN_N - 1 - i; j++)
        {
            if (tmp[j] > tmp[j + 1])
            {
                float t = tmp[j];
                tmp[j]   = tmp[j + 1];
                tmp[j + 1] = t;
            }
        }
    }

    return tmp[MEDIAN_N / 2];
}

/* ================================================================
 *  ADC floating-pin detector
 * ================================================================ */

#define CHECK_WINDOW     20      /* samples to accumulate before judging */
#define STUCK_THRESHOLD  0.001f  /* < 1 mV span -> stuck */
#define FLOAT_THRESHOLD  0.500f  /* > 500 mV span -> floating */

/**
 * @brief  Detect whether an ADC channel is floating, stuck, or healthy.
 * @param  ma   Moving-average instance (for smoothed baseline)
 * @param  raw  Latest raw ADC sample (volts)
 * @return Channel status (CH_OK / CH_FLOATING / CH_STUCK_LOW / CH_STUCK_HIGH)
 *
 * @note   Accumulates CHECK_WINDOW samples before issuing a verdict;
 *         the window then keeps sliding so the status can change
 *         mid-run.
 */
ChannelStatus_t ADC_CheckChannel(MovAvg_t *ma, float raw)
{
    static float history[CHECK_WINDOW];
    static uint8_t h_idx = 0;
    float min_v, max_v, avg;
    uint8_t i;
    float span;

    history[h_idx] = raw;
    h_idx = (h_idx + 1) % CHECK_WINDOW;

    /* compute min/max over the current window */
    min_v = history[0];
    max_v = history[0];
    for (i = 1; i < CHECK_WINDOW; i++)
    {
        if (history[i] < min_v) min_v = history[i];
        if (history[i] > max_v) max_v = history[i];
    }

    span = max_v - min_v;
    avg  = MovAvg_Update(ma, raw);

    if (span < STUCK_THRESHOLD && avg < 0.01f) return CH_STUCK_LOW;
    if (span < STUCK_THRESHOLD && avg > 3.29f) return CH_STUCK_HIGH;
    if (span > FLOAT_THRESHOLD)                 return CH_FLOATING;

    return CH_OK;
}
