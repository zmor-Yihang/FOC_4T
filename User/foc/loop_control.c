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
        speed_loop_div = 0;
        float speed_loop_dt = FOC_CURRENT_LOOP_DT_S * (float)speed_loop_divider;
        handle->target_iq = pid_calculate(handle->pid_speed, handle->target_speed, speed_rpm, speed_loop_dt);
    }

    handle->target_id = 0.0f;

    /* 复用电流闭环 */
    loopControl_run_currentLoop(handle, i_dq, angle_el, speed_rpm);
}

/**
 * @brief 位置闭环运行，位置PD直接输出q轴目标电流，D项使用PLL机械速度避免目标阶跃冲击
 * @param handle                 FOC 控制句柄
 * @param i_dq                   dq 轴电流反馈
 * @param angle_el               电角度 (rad)
 * @param speed_rpm              速度反馈 (RPM)
 * @param position_rad           机械多圈位置反馈 (rad)
 * @param position_loop_divider  位置环相对电流环的分频系数
 */
void loopControl_run_positionLoop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm, float position_rad, uint8_t position_loop_divider)
{
    static uint16_t position_loop_div = 0; // 位置环分频计数器

    /* 位置环按分频执行，未到执行周期时保持上一次 target_iq */
    if (++position_loop_div >= position_loop_divider)
    {
        position_loop_div = 0;                                         // 复位分频计数器
        float position_error = handle->target_position - position_rad; // 位置误差
        float position_speed_rad_s = speed_rpm * (MATH_TWO_PI / 60.0f); // PLL机械速度，单位 rad/s
        float gain_scale = 1.0f;                                        // 接近目标位置时的增益缩放，默认不缩放
        float kp_eff = handle->pid_position->kp;
        float kd_eff = handle->pid_position->kd;

        /* 接近目标位置时按误差比例降低 PID 增益，使收敛更柔和 */
        if (POSITION_PID_GAIN_DEC_ERROR_RAD > 0.0f)
        {
            float position_error_abs = fabsf(position_error);

            if (position_error_abs < POSITION_PID_GAIN_DEC_ERROR_RAD)
            {
                gain_scale = position_error_abs / POSITION_PID_GAIN_DEC_ERROR_RAD;
                kp_eff *= gain_scale;
                kd_eff *= gain_scale;
            }
        }

        /* 计算 PD 项 */
        handle->pid_position->error = position_error;
        handle->pid_position->p_term = kp_eff * handle->pid_position->error;
        handle->pid_position->i_term = 0.0f;
        handle->pid_position->d_term = 0.0f;

        /* D 项取负的反馈速度：位置正向变化时产生反向阻尼 */
        handle->pid_position->derivative = -position_speed_rad_s;
        handle->pid_position->d_term = kd_eff * handle->pid_position->derivative;

        /* 位置环输出限幅为 q 轴目标电流，并保存限幅误差供下一周期抗饱和使用 */
        float out_unclamped = handle->pid_position->p_term + handle->pid_position->i_term + handle->pid_position->d_term;
        handle->pid_position->out = utils_clampf(out_unclamped, handle->pid_position->out_min, handle->pid_position->out_max);

        /* 位置环直接给电流环 q 轴电流目标 */
        handle->target_iq = handle->pid_position->out;
    }

    /* 位置控制当前只使用 q 轴转矩电流，d 轴目标保持为 0 */
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
