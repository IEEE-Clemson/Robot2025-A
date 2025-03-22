#include "pid_control.h"

#include "util.h"

void pid_create_params(struct PIDParams *params, float p, float i, float d)
{
    params->p = p;
    params->i = i;
    params->d = d;

    params->max_output = 1.0f;
    if(i != 0.0f)
        params->max_i_accum = params->max_output / i;
    else
        params->max_i_accum = 0.0f;
}

void pid_controller_init(struct PIDController *pid, struct PIDParams *params)
{
    pid->params = params;
    pid->output = 0.0f;
    pid->setpoint = 0.0f;
    pid->i_accum = 0.0f;
    pid->prev_x = 0.0f;
}

void pid_controller_update(struct PIDController *pid, float x, float dt)
{
    struct PIDParams *params = pid->params;
    float p_term, i_term, d_term;
    float err = pid->setpoint - x;

    pid->i_accum += err * dt;
    pid->i_accum = clamp(pid->i_accum, -params->max_i_accum, params->max_i_accum);

    p_term = params->p * err;
    i_term = params->i * pid->i_accum;
    d_term = params->d * (x - pid->prev_x) / dt;

    pid->output = p_term + i_term + d_term;
    pid->prev_x = x;
}
