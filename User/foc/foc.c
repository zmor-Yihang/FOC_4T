#include "foc.h"

void foc_init(foc_t *handle, pid_controller_t *pid_id, pid_controller_t *pid_iq, pid_controller_t *pid_speed)
{
    handle->target_speed = 0;
    handle->target_id = 0;
    handle->target_iq = 0;

    handle->i_abc.a = 0.0f;
    handle->i_abc.b = 0.0f;
    handle->i_abc.c = 0.0f;

    handle->v_d_out = 0.0f;
    handle->v_q_out = 0.0f;
    handle->i_q_out = 0.0f;

    handle->pid_id = pid_id;
    handle->pid_iq = pid_iq;
    handle->pid_speed = pid_speed;

    handle->duty_cycle.a = 0.0f;
    handle->duty_cycle.b = 0.0f;
    handle->duty_cycle.c = 0.0f;

    handle->angle_offset = 0.0f;
    handle->open_loop_angle_el = 0.0f;
}

void foc_alignment(foc_t *handle)
{
    /* 施加d轴电压，让转子对齐到电角度0位置 */
    dq_t u_dq = {.d = 1.0f, .q = 0.0f};

    /* 开环输出，固定电角度为0 */
    alphabeta_t alpha_beta = ipark_transform(u_dq, 0);
    abc_t duty_abc = svpwm_update(alpha_beta);
    tim_set_pwm_duty(duty_abc.a, duty_abc.b, duty_abc.c);

    /* 等待转子稳定 */
    HAL_Delay(1000);

    /* 读取当前电角度作为零点偏移 */
    handle->angle_offset = encoder_get_angle_rad();

    /* 关闭PWM输出 */
    tim_set_pwm_duty(0.5f, 0.5f, 0.5f);
}

/**
 * @brief 电流闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 */
void foc_current_closed_loop_run(foc_t *handle, dq_t i_dq, float angle_el)
{
    /* 电流环 PID */
    handle->v_d_out = pid_calculate(handle->pid_id, handle->target_id, i_dq.d);
    handle->v_q_out = pid_calculate(handle->pid_iq, handle->target_iq, i_dq.q);

    /* 逆 Park 变换 */
    alphabeta_t v_alphabeta = ipark_transform((dq_t){.d = handle->v_d_out, .q = handle->v_q_out}, angle_el);

    /* SVPWM 输出 */
    handle->duty_cycle = svpwm_update(v_alphabeta);
    tim_set_pwm_duty(handle->duty_cycle.a, handle->duty_cycle.b, handle->duty_cycle.c);
}

/**
 * @brief 速度闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 * @param speed_rpm 速度反馈 (RPM)
 * @param speed_loop_div 速度环分频系数，1表示每次都更新速度环
 */
void foc_speed_closed_loop_run(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm, uint8_t speed_loop_div)
{
    static uint8_t speed_loop_div_cnt = 0;

    if (speed_loop_div == 0)
    {
        speed_loop_div = 1;
    }

    /* 按分频更新速度环，其他周期复用上次 target_iq */
    speed_loop_div_cnt++;
    if (speed_loop_div_cnt >= speed_loop_div)
    {
        speed_loop_div_cnt = 0;
        handle->target_iq = pid_calculate(handle->pid_speed, handle->target_speed, speed_rpm);
    }

    /* Id 目标设为 0  */
    handle->target_id = 0.0f;

    /* 复用电流闭环 */
    foc_current_closed_loop_run(handle, i_dq, angle_el);
}


void foc_set_target_id(foc_t *handle, float id)
{
    handle->target_id = id;
}

void foc_set_target_iq(foc_t *handle, float iq)
{
    handle->target_iq = iq;
}

void foc_set_target_speed(foc_t *handle, float speed_rpm)
{
    handle->target_speed = speed_rpm;
}

void foc_closed_loop_stop(foc_t *handle)
{
    /* 复位所有 PID 控制器，清除积分项 */
    pid_reset(handle->pid_id);
    pid_reset(handle->pid_iq);
    pid_reset(handle->pid_speed);

    /* 清除目标值 */
    handle->target_id = 0.0f;
    handle->target_iq = 0.0f;
    handle->target_speed = 0.0f;

    /* 清除输出 */
    handle->v_d_out = 0.0f;
    handle->v_q_out = 0.0f;

    /* 输出50%占空比，电机停止 */
    tim_set_pwm_duty(0.5f, 0.5f, 0.5f);
}
