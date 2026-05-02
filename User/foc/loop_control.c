#include <math.h>

#include "foc.h"
#include "gate_drive.h"
#include "../adv_alg/weaken_flux.h"

/**
 * @brief 电流闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 * @param speed_rpm 机械速度反馈 (RPM), 仅在启用前馈解耦时使用
 */
void loopControl_run_currentLoop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm)
{
    float v_d_pi;
    float v_q_pi;
    float v_d_ff;
    float v_q_ff;
    float v_d_unsat;
    float v_q_unsat;

    /* 电流环 PI */
    v_d_pi = pid_calculate(handle->pid_id, handle->target_id, i_dq.d, FOC_CURRENT_LOOP_DT_S);
    v_q_pi = pid_calculate(handle->pid_iq, handle->target_iq, i_dq.q, FOC_CURRENT_LOOP_DT_S);

#if (FOC_DECOUPLING_ENABLE == 1)
    /* 前馈解耦 */
    float omega_e = speed_rpm * (MATH_TWO_PI / 60.0f) * MOTOR_POLE_PAIRS; /* 电角速度 rad/s */

    v_d_ff = -omega_e * MOTOR_LQ_H * i_dq.q;
    v_q_ff = omega_e * (MOTOR_LD_H * i_dq.d + MOTOR_PSI_F);
#else
    /* 关闭前馈解耦 */
    (void)speed_rpm;
    v_d_ff = 0.0f;
    v_q_ff = 0.0f;
#endif /* FOC_DECOUPLING_ENABLE */

    v_d_unsat = v_d_pi + v_d_ff;
    v_q_unsat = v_q_pi + v_q_ff;

    handle->v_d_pi = v_d_pi;
    handle->v_q_pi = v_q_pi;
    handle->v_d_ff = v_d_ff;
    handle->v_q_ff = v_q_ff;
    handle->v_d_cmd = v_d_unsat;
    handle->v_q_cmd = v_q_unsat;
    handle->v_d_out = v_d_unsat;
    handle->v_q_out = v_q_unsat;

    /* dq 电压矢量联合限幅，参考 VESC 开源代码 */
    float v_limit = U_DC * FOC_VOLTAGE_LIMIT_SVPWM_SCALE;
    float v_mag_sq = handle->v_d_out * handle->v_d_out + handle->v_q_out * handle->v_q_out;
    float v_limit_sq = v_limit * v_limit;

    if (v_mag_sq > v_limit_sq)
    {
        float scale = v_limit / sqrtf(v_mag_sq);
        handle->v_d_out *= scale;
        handle->v_q_out *= scale;
    }

    handle->pid_id->backcalc_error = v_d_unsat - handle->v_d_out;
    handle->pid_iq->backcalc_error = v_q_unsat - handle->v_q_out;

    /* 逆 Park 变换后交由输出层完成调制与PWM下发 */
    alphabeta_t v_alphabeta = ipark_transform((dq_t){.d = handle->v_d_out, .q = handle->v_q_out}, angle_el);
    handle->duty_cycle = gateDrive_set_voltage(v_alphabeta);
}

/**
 * @brief 速度闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 * @param speed_rpm 速度反馈 (RPM)
 * @param speed_loop_divider 速度环相对电流环的分频系数
 */
void loopControl_run_speedLoop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm, uint8_t speed_loop_divider)
{
    static uint8_t speed_loop_div = 0;

    /* 速度环按分频执行，其他周期保持上一次 iq 目标 */
    if (++speed_loop_div >= speed_loop_divider)
    {
        float speed_loop_dt = FOC_CURRENT_LOOP_DT_S * (float)speed_loop_divider;
        speed_loop_div = 0;
        handle->target_iq = pid_calculate(handle->pid_speed, handle->target_speed, speed_rpm, speed_loop_dt);
    }

    handle->target_id = 0.0f;

    /* 复用电流闭环 */
    loopControl_run_currentLoop(handle, i_dq, angle_el, speed_rpm);
}

/**
 * @brief 位置闭环运行，位置PD直接输出q轴目标电流，D项使用测量微分避免目标阶跃冲击
 * @param handle                 FOC 控制句柄
 * @param i_dq                   dq 轴电流反馈
 * @param angle_el               电角度 (rad)
 * @param speed_rpm              速度反馈 (RPM)
 * @param position_rad           机械多圈位置反馈 (rad)
 * @param position_loop_divider  位置环相对电流环的分频系数
 */
void loopControl_run_positionLoop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm, float position_rad, uint8_t position_loop_divider)
{
    static uint16_t position_loop_div = 0;
    static float prev_position_rad = 0.0f;
    static uint8_t position_feedback_valid = 0U;
    static float position_speed_filtered_rad_s = 0.0f;
    uint16_t position_total_divider = (uint16_t)position_loop_divider;

    if (position_total_divider == 0U)
    {
        position_total_divider = 1U;
    }

    if (++position_loop_div >= position_total_divider)
    {
        float position_error = handle->target_position - position_rad; // 位置误差
        float position_loop_dt = FOC_CURRENT_LOOP_DT_S * (float)position_total_divider;
        position_loop_div = 0;

        if (fabsf(position_error) <= POSITION_DEADBAND_RAD)
        {
            handle->target_speed = 0.0f;
            handle->target_iq = 0.0f;
            if (handle->pid_position != NULL)
            {
                pid_reset(handle->pid_position);
            }
        }
        else if (handle->pid_position != NULL)
        {
            pid_controller_t *pid = handle->pid_position;
            float position_speed_rad_s = 0.0f;
            float gain_scale = 1.0f;
            float kp_eff = pid->kp;
            float ki_eff = pid->ki;
            float kd_eff = pid->kd;

            if (position_feedback_valid != 0U)
            {
                position_speed_rad_s = (position_rad - prev_position_rad) / position_loop_dt;
            }

            position_speed_filtered_rad_s += POSITION_PID_D_FILTER_ALPHA * (position_speed_rad_s - position_speed_filtered_rad_s);

            if (POSITION_PID_GAIN_DEC_ERROR_RAD > 0.0f)
            {
                float position_error_abs = fabsf(position_error);

                if (position_error_abs < POSITION_PID_GAIN_DEC_ERROR_RAD)
                {
                    gain_scale = position_error_abs / POSITION_PID_GAIN_DEC_ERROR_RAD;
                    kp_eff *= gain_scale;
                    ki_eff *= gain_scale;
                    kd_eff *= gain_scale;
                }
            }

            pid->error = position_error;
            pid->p_term = kp_eff * pid->error;
            pid->i_term = 0.0f;
            pid->d_term = 0.0f;

            if ((pid->mode == PID_MODE_PI) || (pid->mode == PID_MODE_PID))
            {
                float integral_increment = (kp_eff * ki_eff * pid->error - pid->kt * pid->backcalc_error) * position_loop_dt;

                pid->integral += integral_increment;
                pid->integral = utils_clampf(pid->integral, -pid->integral_max, pid->integral_max);
                pid->i_term = pid->integral;
            }

            if (pid->mode == PID_MODE_PID)
            {
                pid->derivative = -position_speed_filtered_rad_s;
                pid->d_term = kd_eff * pid->derivative;
            }
            else
            {
                pid->derivative = 0.0f;
            }

            float out_unclamped = pid->p_term + pid->i_term + pid->d_term;
            pid->out = utils_clampf(out_unclamped, pid->out_min, pid->out_max);
            pid->backcalc_error = out_unclamped - pid->out;
            pid->prev_error = pid->error;

            handle->target_iq = pid->out;
            handle->target_speed = 0.0f;
        }
        else
        {
            handle->target_speed = 0.0f;
            handle->target_iq = 0.0f;
        }

        prev_position_rad = position_rad;
        position_feedback_valid = 1U;
    }

    handle->target_id = 0.0f;

    /* 复用电流闭环 */
    loopControl_run_currentLoop(handle, i_dq, angle_el, speed_rpm);
}

/**
 * @brief 带弱磁的速度闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 * @param speed_rpm 速度反馈 (RPM)
 * @param speed_loop_divider 速度环相对电流环的分频系数
 */
void loopControl_run_speedWeakLoop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm, uint8_t speed_loop_divider)
{
    static uint8_t speed_loop_div = 0;

    /* 速度环按分频执行，其他周期保持上一次 iq 目标 */
    if (++speed_loop_div >= speed_loop_divider)
    {
        float speed_loop_dt = FOC_CURRENT_LOOP_DT_S * (float)speed_loop_divider;
        speed_loop_div = 0;
        handle->target_iq = pid_calculate(handle->pid_speed, handle->target_speed, speed_rpm, speed_loop_dt);
    }

    if (handle->flux_weak != NULL)
    {
        /* 弱磁环目前使用限幅前电压请求值作为电压裕量反馈。 */
        handle->target_id = fluxWeak_calculate(handle->flux_weak, handle->v_d_cmd, handle->v_q_cmd, FOC_CURRENT_LOOP_DT_S);
    }
    else
    {
        handle->target_id = 0.0f;
    }

    /* 复用电流闭环 */
    loopControl_run_currentLoop(handle, i_dq, angle_el, speed_rpm);
}


