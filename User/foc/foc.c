#include "foc.h"

#define FOC_VOLTAGE_LIMIT_SVPWM_SCALE (0.57735026919f)

static float foc_wrap_angle(float angle)
{
    while (angle >= ENCODER_TWO_PI)
    {
        angle -= ENCODER_TWO_PI;
    }

    while (angle < 0.0f)
    {
        angle += ENCODER_TWO_PI;
    }

    return angle;
}

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
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    uint16_t sample_idx;
    uint8_t repeat_idx;

    dq_t u_dq = {.d = FOC_ALIGN_D_AXIS_VOLTAGE, .q = 0.0f};

    /* 先建立稳定磁场 */
    {
        alphabeta_t alpha_beta = ipark_transform(u_dq, 0.0f);
        abc_t duty_abc = svpwm_update(alpha_beta);
        tim_set_pwm_duty(duty_abc.a, duty_abc.b, duty_abc.c);
    }

    HAL_Delay(FOC_ALIGN_SETTLE_TIME_MS);

    /* 缓慢扫描一整圈电角度，累计每个采样点的偏移圆均值 */
    for (repeat_idx = 0U; repeat_idx < FOC_ALIGN_SCAN_REPEAT; repeat_idx++)
    {
        for (sample_idx = 0U; sample_idx < FOC_ALIGN_SCAN_POINTS; sample_idx++)
        {
            float angle_cmd = (ENCODER_TWO_PI * (float)sample_idx) / (float)FOC_ALIGN_SCAN_POINTS;
            float angle_meas;
            float offset_sample;
            float sin_offset;
            float cos_offset;
            alphabeta_t alpha_beta = ipark_transform(u_dq, angle_cmd);
            abc_t duty_abc = svpwm_update(alpha_beta);

            tim_set_pwm_duty(duty_abc.a, duty_abc.b, duty_abc.c);

            HAL_Delay(FOC_ALIGN_SAMPLE_INTERVAL_MS);

            angle_meas = encoder_get_angle_rad();
            offset_sample = foc_wrap_angle(angle_cmd - angle_meas);

            fast_sin_cos(offset_sample, &sin_offset, &cos_offset);
            sin_sum += sin_offset;
            cos_sum += cos_offset;
        }

        /* 每圈结束后回到零角，减少下一圈起点跳变 */
        {
            alphabeta_t alpha_beta = ipark_transform(u_dq, 0.0f);
            abc_t duty_abc = svpwm_update(alpha_beta);
            tim_set_pwm_duty(duty_abc.a, duty_abc.b, duty_abc.c);
            HAL_Delay(FOC_ALIGN_SETTLE_TIME_MS);
        }
    }

    handle->angle_offset = foc_wrap_angle(atan2f(sin_sum, cos_sum));

    /* 关闭PWM输出 */
    tim_set_pwm_duty(0.5f, 0.5f, 0.5f);
}

/**
 * @brief 电流闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 */
void foc_current_loop_run(foc_t *handle, dq_t i_dq, float angle_el)
{
    float v_d_unsat;
    float v_q_unsat;

    /* 电流环 PID */
    v_d_unsat = pid_calculate(handle->pid_id, handle->target_id, i_dq.d);
    v_q_unsat = pid_calculate(handle->pid_iq, handle->target_iq, i_dq.q);

    handle->v_d_out = v_d_unsat;
    handle->v_q_out = v_q_unsat;

    /* dq 电压矢量联合限幅，适配 SVPWM */
    {
        float v_limit = U_DC * FOC_VOLTAGE_LIMIT_SVPWM_SCALE;
        float v_mag_sq = handle->v_d_out * handle->v_d_out + handle->v_q_out * handle->v_q_out;
        float v_limit_sq = v_limit * v_limit;

        if (v_mag_sq > v_limit_sq)
        {
            float scale = v_limit / sqrtf(v_mag_sq);
            handle->v_d_out *= scale;
            handle->v_q_out *= scale;
        }
    }

    handle->pid_id->backcalc_error = v_d_unsat - handle->v_d_out;
    handle->pid_iq->backcalc_error = v_q_unsat - handle->v_q_out;

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
 */
void foc_speed_loop_run(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm, uint8_t speed_loop_divider)
{
    static uint8_t speed_loop_div = 0;
    uint8_t divider = (speed_loop_divider == 0U) ? 1U : speed_loop_divider;

    /* 速度环按分频执行，其他周期保持上一次 iq 目标 */
    if (++speed_loop_div >= divider)
    {
        speed_loop_div = 0;
        handle->target_iq = pid_calculate(handle->pid_speed, handle->target_speed, speed_rpm);
    }

    /* Id 目标设为 0  */
    handle->target_id = 0.0f;

    /* 复用电流闭环 */
    foc_current_loop_run(handle, i_dq, angle_el);
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
