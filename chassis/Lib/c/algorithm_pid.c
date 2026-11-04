//
// Created by 31466 on 2025/12/15.
//

#include "../inc/algorithm_pid.h"

#include <stdlib.h>


/* Internal clamp helper */
float clamp(float val, float limit)
{
    if (val > limit)  return limit;
    if (val < -limit) return -limit;
    return val;
}

void PID_Init(PID_t *pid,
              float kp, float ki, float kd,
              float dt,
              float integral_limit,
              float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;

    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;

    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    pid->error = 0.0f;
    pid->error_last = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;
}

float PID_Update(PID_t *pid, float target, float measurement)
{
    pid->error = target - measurement;

    /* Integral term with anti-windup */
    pid->integral += pid->error * pid->dt;
    pid->integral = clamp(pid->integral, pid->integral_limit);

    /* Derivative term */
    float a = (pid->error - pid->error_last) / pid->dt;

    /* PID sum */
    pid->out =
        pid->kp * pid->error +
        pid->ki * pid->integral +
        pid->kd * a;

    pid->out = clamp(pid->out, pid->output_limit);

    pid->error_last = pid->error;
    return pid->out;
}
