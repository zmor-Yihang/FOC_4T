#include <math.h>

#include "foc.h"
#include "gate_drive.h"
#include "../sensor/encoder.h"

void foc_init(foc_t *handle)
{
    // FOC 指令初始化
    handle->cmd.mode = FOC_MODE_CURRENT;
    handle->cmd.target_speed = 0.0f;
    handle->cmd.target_position = 0.0f;
    handle->cmd.target_id = 0.0f;
    handle->cmd.target_iq = 0.0f;

    // FOC 反馈量初始化
    handle->feedback.angle_el = 0.0f;
    handle->feedback.speed_rpm = 0.0f;
    handle->feedback.i_dq.d = 0.0f;
    handle->feedback.i_dq.q = 0.0f;
    handle->feedback.position_rad = 0.0f;

    // FOC 内部状态初始化
    handle->state.v_d_cmd = 0.0f;
    handle->state.v_q_cmd = 0.0f;
    handle->state.v_d_out = 0.0f;
    handle->state.v_q_out = 0.0f;
    handle->state.v_d_pi = 0.0f;
    handle->state.v_q_pi = 0.0f;
    handle->state.v_d_ff = 0.0f;
    handle->state.v_q_ff = 0.0f;

    // 初始化 PWM 输出为 0
    handle->state.duty_cycle.a = 0.0f;
    handle->state.duty_cycle.b = 0.0f;
    handle->state.duty_cycle.c = 0.0f;

#if (FLUX_WEAK_ENABLE == 1)
    // 弱磁内部状态初始化
    pid_init(&handle->flux_weak_pid, PID_ROLE_SPEED, FLUX_WEAK_PID_MODE, FLUX_WEAK_KP, FLUX_WEAK_KI, FLUX_WEAK_KD, FLUX_WEAK_ID_MIN, 0.0f);
    handle->flux_weak_u_current_filtered = 0.0f;
#endif /* FLUX_WEAK_ENABLE */

    // 编码器零点偏移初始为0，需通过 foc_alignment_zero() 函数校准
    handle->angle_offset = 0.0f;
    
    // 初始化分频计数器
    handle->speed_loop_cnt = 0;
    handle->position_loop_cnt = 0;
}

void foc_alignment_zero(foc_t *handle)
{
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    uint16_t sample_idx;
    uint8_t repeat_idx;
    dq_t u_dq = {.d = FOC_ALIGN_D_AXIS_VOLTAGE, .q = 0.0f};

    /* 把电机拉到 d 轴 */
    handle->state.duty_cycle = gateDrive_set_voltage(ipark_transform(u_dq, 0.0f));
    HAL_Delay(FOC_ALIGN_SETTLE_TIME_MS);

    /* 缓慢扫描一整圈电角度，累计每个采样点的偏移圆均值 */
    for (repeat_idx = 0U; repeat_idx < FOC_ALIGN_SCAN_REPEAT; repeat_idx++)
    {
        for (sample_idx = 0U; sample_idx < FOC_ALIGN_SCAN_POINTS; sample_idx++)
        {
            float angle_cmd = (MATH_TWO_PI * (float)sample_idx) / (float)FOC_ALIGN_SCAN_POINTS;
            float angle_meas;
            float offset_sample;
            float sin_offset;
            float cos_offset;

            handle->state.duty_cycle = gateDrive_set_voltage(ipark_transform(u_dq, angle_cmd));
            HAL_Delay(FOC_ALIGN_SAMPLE_INTERVAL_MS);

            encoder_update();
            HAL_Delay(1); /* 确保角度读取完成 */
            angle_meas = encoder_get_encoderAngle();
            offset_sample = wrap_0_2pi(angle_meas - angle_cmd);

            fast_sin_cos(offset_sample, &sin_offset, &cos_offset);
            sin_sum += sin_offset;
            cos_sum += cos_offset;
        }

        /* 每圈结束后回到零角，减少下一圈起点跳变 */
        handle->state.duty_cycle = gateDrive_set_voltage(ipark_transform(u_dq, 0.0f));
        HAL_Delay(FOC_ALIGN_SETTLE_TIME_MS);
    }

    handle->angle_offset = wrap_0_2pi(atan2f(sin_sum, cos_sum) - FOC_ELEC_ANGLE_TRIM_RAD);

    HAL_Delay(1);     /* 确保I2C通信完成 */
    encoder_update(); /* 刷新PLL状态 */

    /* 关闭PWM输出 */
    handle->state.duty_cycle = gateDrive_stop();
}

/**
 * @brief FOC 统一控制步进函数
 * @param handle                 FOC 控制句柄，保存指令、反馈、控制器和内部状态
 * @param speed_loop_divider     速度环相对电流环的分频系数；为 0 时按每次都执行处理
 * @param position_loop_divider  位置环分频系数；为 0 时按每次都执行处理
 */
void foc_step(foc_t *handle, uint8_t speed_loop_divider, uint8_t position_loop_divider)
{
    // 根据分频系数换算慢环实际执行周期，PI控制器要使用对应 dt
    float speed_loop_dt = FOC_CURRENT_LOOP_DT_S * (float)speed_loop_divider;
    float position_loop_dt = speed_loop_dt * (float)position_loop_divider;

    /* 电流环和电压输出计算过程中使用的局部变量 */
    dq_t i_dq;                  // 当前采样得到的 d/q 轴电流反馈
    float speed_rpm;            // 当前机械转速反馈，单位 rpm
    float angle_el;             // 当前电角度，供逆 Park 变换使用
    float v_d_pi;               // d 轴电流环 PI 输出电压
    float v_q_pi;               // q 轴电流环 PI 输出电压
    float v_d_ff;               // d 轴前馈解耦补偿电压
    float v_q_ff;               // q 轴前馈解耦补偿电压
    float v_d_unsat;            // d 轴限幅前电压指令
    float v_q_unsat;            // q 轴限幅前电压指令
    float v_limit;              // SVPWM 线性区允许的最大电压矢量幅值
    float v_mag_sq;             // 当前 dq 电压矢量幅值平方
    float v_limit_sq;           // 电压限幅阈值平方，用于避免不必要的开方
    alphabeta_t v_alphabeta;    // 逆 Park 后的 alpha/beta 静止坐标系电压

    /* 先根据当前控制模式调度外环，最终得到电流环目标 target_id/target_iq */
    switch (handle->cmd.mode)
    {
    case FOC_MODE_POSITION: // 位置环：位置误差 -> 目标 q 轴电流
        if (divider_ready(&handle->position_loop_cnt, position_loop_divider))
        {
            float position_error;

            position_error = handle->cmd.target_position - handle->feedback.position_rad;

            /* 进入位置死区后直接清零力矩电流，并清空位置环积分，避免静止时残留输出 */
            if (fabsf(position_error) <= POSITION_DEADBAND_RAD)
            {
                handle->cmd.target_speed = 0.0f;
                handle->cmd.target_iq = 0.0f;
                pid_reset(&handle->controller.position);
                pid_reset(&handle->controller.speed);
            }
            else
            {
                /* 仿照 VESC 的位置闭环思路：位置环直接输出力矩电流请求，跳过速度环级联 */
                handle->cmd.target_iq = pid_calculate(&handle->controller.position, handle->cmd.target_position, handle->feedback.position_rad, position_loop_dt);
            }
        }

        /* 位置直驱电流模式下不使用速度环，target_speed 仅保留给调试观察。 */
        handle->cmd.target_speed = 0.0f;

        /* 位置模式不参与弱磁，避免位置控制到位/低速时引入额外 d 轴电流。 */
        handle->cmd.target_id = 0.0f;
        break;

    case FOC_MODE_SPEED:
        /* 普通速度模式：速度环输出 q 轴目标电流 */
        if (divider_ready(&handle->speed_loop_cnt, speed_loop_divider))
        {
            handle->cmd.target_iq = pid_calculate(&handle->controller.speed, handle->cmd.target_speed, handle->feedback.speed_rpm, speed_loop_dt);
        }

#if (FLUX_WEAK_ENABLE == 1)
        /* 弱磁开启时自动产生负 d 轴电流用于扩速。 */
        float flux_weak_u_mag = sqrtf(handle->state.v_d_cmd * handle->state.v_d_cmd + handle->state.v_q_cmd * handle->state.v_q_cmd);
        float flux_weak_alpha = FLUX_WEAK_VOLTAGE_FILTER_CONST;
        float flux_weak_u_ref = (U_DC * FOC_VOLTAGE_LIMIT_SVPWM_SCALE) * FLUX_WEAK_U_REF_RATIO;

        if (flux_weak_alpha < 0.0f)
        {
            flux_weak_alpha = 0.0f;
        }
        else if (flux_weak_alpha > 1.0f)
        {
            flux_weak_alpha = 1.0f;
        }

        handle->flux_weak_u_current_filtered = handle->flux_weak_u_current_filtered * (1.0f - flux_weak_alpha) + flux_weak_u_mag * flux_weak_alpha;
        handle->cmd.target_id = pid_calculate(&handle->flux_weak_pid, flux_weak_u_ref, handle->flux_weak_u_current_filtered, FOC_CURRENT_LOOP_DT_S);
#else
        handle->cmd.target_id = 0.0f;
#endif /* FLUX_WEAK_ENABLE */
        break;

    case FOC_MODE_CURRENT:
    default:
        /* 电流模式下 target_id/target_iq 由外部设置，这里不修改电流目标 */
        break;
    }

    /* 读取本周期反馈量快照，避免后续计算过程中多次访问结构体成员 */
    i_dq = handle->feedback.i_dq;
    speed_rpm = handle->feedback.speed_rpm;
    angle_el = handle->feedback.angle_el;

    /* 电流环每次 foc_step() 都执行：d/q 轴电流误差 -> d/q 轴电压 PI 输出 */
    v_d_pi = pid_calculate(&handle->controller.id, handle->cmd.target_id, i_dq.d, FOC_CURRENT_LOOP_DT_S);
    v_q_pi = pid_calculate(&handle->controller.iq, handle->cmd.target_iq, i_dq.q, FOC_CURRENT_LOOP_DT_S);

#if (FOC_DECOUPLING_ENABLE == 1)
    /* 前馈解耦：根据电角速度补偿交叉耦合项，减小高速时 d/q 轴互相影响 */
    float omega_e = speed_rpm * (MATH_TWO_PI / 60.0f) * MOTOR_POLE_PAIRS; /* 电角速度 rad/s */

    v_d_ff = -omega_e * MOTOR_LQ_H * i_dq.q;
    v_q_ff = omega_e * (MOTOR_LD_H * i_dq.d + MOTOR_PSI_F);
#else
    /* 关闭前馈解耦时，仅使用 PI 输出作为电压指令 */
    v_d_ff = 0.0f;
    v_q_ff = 0.0f;
#endif /* FOC_DECOUPLING_ENABLE */

    /* PI 输出与前馈补偿相加，得到限幅前的 dq 电压请求 */
    v_d_unsat = v_d_pi + v_d_ff;
    v_q_unsat = v_q_pi + v_q_ff;

    /* 保存调试状态：cmd 表示限幅前请求，out 表示当前准备输出的电压 */
    handle->state.v_d_pi = v_d_pi;
    handle->state.v_q_pi = v_q_pi;
    handle->state.v_d_ff = v_d_ff;
    handle->state.v_q_ff = v_q_ff;
    handle->state.v_d_cmd = v_d_unsat;
    handle->state.v_q_cmd = v_q_unsat;
    handle->state.v_d_out = v_d_unsat;
    handle->state.v_q_out = v_q_unsat;

    /* dq 电压矢量联合限幅，保证输出电压矢量不超过 SVPWM 在线性区可用电压 */
    v_limit = U_DC * FOC_VOLTAGE_LIMIT_SVPWM_SCALE;
    v_mag_sq = handle->state.v_d_out * handle->state.v_d_out + handle->state.v_q_out * handle->state.v_q_out;
    v_limit_sq = v_limit * v_limit;

    if (v_mag_sq > v_limit_sq)
    {
        /* 超限时保持电压矢量方向不变，只按比例缩小幅值 */
        float scale = v_limit / sqrtf(v_mag_sq);
        handle->state.v_d_out *= scale;
        handle->state.v_q_out *= scale;
    }

    /* 将限幅造成的电压差反馈给 PID，用于抗积分饱和反算 */
    handle->controller.id.backcalc_error = v_d_unsat - handle->state.v_d_out;
    handle->controller.iq.backcalc_error = v_q_unsat - handle->state.v_q_out;

    /* 逆 Park 变换：dq 旋转坐标系电压 -> alpha/beta 静止坐标系电压 */
    v_alphabeta = ipark_transform((dq_t){.d = handle->state.v_d_out, .q = handle->state.v_q_out}, angle_el);

    /* 输出层完成 SVPWM 调制和 PWM 占空比下发，并返回实际占空比用于调试观察 */
    handle->state.duty_cycle = gateDrive_set_voltage(v_alphabeta);
}