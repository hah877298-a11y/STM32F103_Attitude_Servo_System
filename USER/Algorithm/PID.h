#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

/**
 * @file  PID.h
 * @brief Position-form PID controller interface
 */

/**
 * @brief  PID tunable gains
 */
typedef struct
{
    float Kp;       /* proportional gain */
    float Ki;       /* integral gain */
    float Kd;       /* derivative gain */
} PID_Params;

/**
 * @brief  PID runtime state (updated on each call)
 */
typedef struct
{
    float setpoint;         /* target value */
    float prev_error;       /* previous error (derivative term) */
    float integral;         /* accumulated integral */
    float integral_limit;   /* integral clamp (positive) */
    float output_min;       /* output lower bound */
    float output_max;       /* output upper bound */
} PID_State;

void PID_Init(PID_State *state, float integral_limit,
              float output_min, float output_max);

/**
 * @brief  Compute one PID output (clamped)
 * @return Control output in [output_min, output_max]
 */
float PID_Compute(PID_State *state, const PID_Params *params,
                  float measurement, float dt);

/**
 * @brief  Set setpoint (integral reset)
 */
void PID_SetSetpoint(PID_State *state, float setpoint);

/**
 * @brief  Reset PID state
 */
void PID_Reset(PID_State *state);

#endif /* __PID_H */
