#include "PID.h"

/**
 * @file  PID.c
 * @brief Position-form PID controller implementation
 */

/**
 * @brief  Initialize PID controller state
 * @param  state           PID state
 * @param  integral_limit  Integral clamp limit (>0)
 * @param  output_min      Output lower bound
 * @param  output_max      Output upper bound
 */
void PID_Init(PID_State *state, float integral_limit,
              float output_min, float output_max)
{
    state->setpoint       = 0.0f;
    state->prev_error     = 0.0f;
    state->integral       = 0.0f;
    state->integral_limit = integral_limit;
    state->output_min     = output_min;
    state->output_max     = output_max;
}

/**
 * @brief  Compute one PID control output
 * @param  state        PID state (integral, prev error updated)
 * @param  params       PID gains (Kp, Ki, Kd)
 * @param  measurement  Current measurement
 * @param  dt           Time since last call (s)
 * @return Output clamped to [output_min, output_max]
 */
float PID_Compute(PID_State *state, const PID_Params *params,
                  float measurement, float dt)
{
    float error, derivative, output;

    error = state->setpoint - measurement;

    state->integral += error * dt;

    /* anti-windup: clamp integral */
    if (state->integral > state->integral_limit)
    {
        state->integral = state->integral_limit;
    }
    else if (state->integral < -state->integral_limit)
    {
        state->integral = -state->integral_limit;
    }

    if (dt > 0.0001f)  /* guard: skip derivative on invalid dt */
    {
        derivative = (error - state->prev_error) / dt;
    }
    else
    {
        derivative = 0.0f;
    }

    output = params->Kp * error
           + params->Ki * state->integral
           + params->Kd * derivative;

    /* clamp output; integral already clamped above */
    if (output > state->output_max)
    {
        output = state->output_max;
    }
    else if (output < state->output_min)
    {
        output = state->output_min;
    }

    state->prev_error = error;

    return output;
}

/**
 * @brief  Set the PID setpoint (resets integral to avoid overshoot)
 * @param  state     PID state
 * @param  setpoint  New target value
 */
void PID_SetSetpoint(PID_State *state, float setpoint)
{
    state->setpoint = setpoint;
    state->integral = 0.0f;
}

/**
 * @brief  Reset PID state (integral and prev error cleared)
 * @param  state  PID state
 *
 * @note   Setpoint is retained.
 */
void PID_Reset(PID_State *state)
{
    state->prev_error = 0.0f;
    state->integral   = 0.0f;
}
