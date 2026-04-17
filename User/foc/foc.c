#include "foc.h"

void foc_init(foc_t *handle, pid_controller_t *pid_id, pid_controller_t *pid_iq, pid_controller_t *pid_speed)
{
    handle->target_speed = 0.0f;
    handle->target_id = 0.0f;
    handle->target_iq = 0.0f;

    handle->v_d_out = 0.0f;
    handle->v_q_out = 0.0f;
    handle->v_d_pi = 0.0f;
    handle->v_q_pi = 0.0f;
    handle->v_d_ff = 0.0f;
    handle->v_q_ff = 0.0f;

    handle->pid_id = pid_id;
    handle->pid_iq = pid_iq;
    handle->pid_speed = pid_speed;

    handle->duty_cycle.a = 0.0f;
    handle->duty_cycle.b = 0.0f;
    handle->duty_cycle.c = 0.0f;

    handle->angle_offset = 0.0f;
}

void foc_set_id(foc_t *handle, float id)
{
    handle->target_id = id;
}

void foc_set_iq(foc_t *handle, float iq)
{
    handle->target_iq = iq;
}

void foc_set_speed(foc_t *handle, float speed_rpm)
{
    handle->target_speed = speed_rpm;
}

