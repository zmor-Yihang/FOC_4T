#include "foc.h"

#define FOC_CURRENT_LOOP_DT_S (1.0f / FOC_CURRENT_LOOP_FREQ_HZ)

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
    handle->v_d_pi = 0.0f;
    handle->v_q_pi = 0.0f;
    handle->v_d_ff = 0.0f;
    handle->v_q_ff = 0.0f;
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

    // 把电机拉到d轴
    alphabeta_t alpha_beta = ipark_transform(u_dq, 0.0f);
    abc_t duty_abc = svpwm_update(alpha_beta);
    tim_set_pwmDuty(duty_abc.a, duty_abc.b, duty_abc.c);

    HAL_Delay(FOC_ALIGN_SETTLE_TIME_MS);

    /* 缓慢扫描一整圈电角度，累计每个采样点的偏移圆均值 */
    for (repeat_idx = 0; repeat_idx < FOC_ALIGN_SCAN_REPEAT; repeat_idx++)
    {
        for (sample_idx = 0U; sample_idx < FOC_ALIGN_SCAN_POINTS; sample_idx++)
        {
            // 相邻两个采样点的电角度
            float angle_cmd = (ENCODER_TWO_PI * (float)sample_idx) / (float)FOC_ALIGN_SCAN_POINTS;
            float angle_meas;    // 电角度测量值
            float offset_sample; // 偏移值
            float sin_offset;
            float cos_offset;
            alphabeta_t alpha_beta = ipark_transform(u_dq, angle_cmd);
            abc_t duty_abc = svpwm_update(alpha_beta);

            tim_set_pwmDuty(duty_abc.a, duty_abc.b, duty_abc.c);

            HAL_Delay(FOC_ALIGN_SAMPLE_INTERVAL_MS);

            encoder_update();
            HAL_Delay(1); // 
            angle_meas = encoder_get_encoderAngle();
            offset_sample = angle_wrap_0_2pi(angle_meas - angle_cmd);

            fast_sin_cos(offset_sample, &sin_offset, &cos_offset);
            sin_sum += sin_offset;
            cos_sum += cos_offset;
        }

        /* 每圈结束后回到零角，减少下一圈起点跳变 */
        alphabeta_t alpha_beta = ipark_transform(u_dq, 0.0f);
        abc_t duty_abc = svpwm_update(alpha_beta);
        tim_set_pwmDuty(duty_abc.a, duty_abc.b, duty_abc.c);
        HAL_Delay(FOC_ALIGN_SETTLE_TIME_MS);
    }

    /* 灏嗗浐瀹氱數瑙掕ˉ鍋垮悎骞跺埌闆剁偣鍋忕Щ锛屽悗缁帶鍒惰璁＄畻鍙洿鎺ュ鐢?*/
    handle->angle_offset = angle_wrap_0_2pi(atan2f(sin_sum, cos_sum) - FOC_ELEC_ANGLE_TRIM_RAD);

    HAL_Delay(1);     // 确保I2C通信完成
    encoder_update(); // 刷新PLL状态

    /* 关闭PWM输出 */
    tim_set_pwmDuty(0.5f, 0.5f, 0.5f);
}

/**
 * @brief 电流闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 */
void foc_run_currentLoop(foc_t *handle, dq_t i_dq, float angle_el)
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
    /* 前馈解耦（基于PMSM dq模型） */
    float speed_rpm = encoder_get_speed();
    float omega_e = speed_rpm * (ENCODER_TWO_PI / 60.0f) * MOTOR_POLE_PAIRS; // 电角速度 rad/s

    v_d_ff = -omega_e * MOTOR_LQ_H * i_dq.q;
    v_q_ff = omega_e * (MOTOR_LD_H * i_dq.d + MOTOR_PSI_F);
#else
    /* 关闭前馈解耦 */
    v_d_ff = 0.0f;
    v_q_ff = 0.0f;
#endif /* FOC_DECOUPLING_ENABLE */

    v_d_unsat = v_d_pi + v_d_ff;
    v_q_unsat = v_q_pi + v_q_ff;

    handle->v_d_pi = v_d_pi;
    handle->v_q_pi = v_q_pi;
    handle->v_d_ff = v_d_ff;
    handle->v_q_ff = v_q_ff;
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

    /* 逆 Park 变换 */
    alphabeta_t v_alphabeta = ipark_transform((dq_t){.d = handle->v_d_out, .q = handle->v_q_out}, angle_el);

    /* SVPWM 输出 */
    handle->duty_cycle = svpwm_update(v_alphabeta);
    tim_set_pwmDuty(handle->duty_cycle.a, handle->duty_cycle.b, handle->duty_cycle.c);
}

/**
 * @brief 速度闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 * @param speed_rpm 速度反馈 (RPM)
 */
void foc_run_speedLoop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm, uint8_t speed_loop_divider)
{
    static uint8_t speed_loop_div = 0;

    /* 速度环按分频执行，其他周期保持上一次 iq 目标 */
    if (++speed_loop_div >= speed_loop_divider)
    {
        float speed_loop_dt = FOC_CURRENT_LOOP_DT_S * (float)speed_loop_divider;
        speed_loop_div = 0;
        handle->target_iq = pid_calculate(handle->pid_speed, handle->target_speed, speed_rpm, speed_loop_dt);
    }

    /* Id 目标设为 0  */
    handle->target_id = 0.0f;

    /* 复用电流闭环 */
    foc_run_currentLoop(handle, i_dq, angle_el);
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
