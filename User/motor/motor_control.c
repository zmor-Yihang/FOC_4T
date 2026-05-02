#include "motor_control.h"

/*
 * 电机控制总入口：
 * 1. 根据 FOC 模式选择电流环、速度环或位置环闭环。
 * 2. ADC 注入转换完成中断作为 FOC 实时控制节拍。
 * 3. 编码器 / 观测器提供电角度与速度反馈，foc_step() 完成环路计算与 SVPWM 输出。
 */
static foc_t motor_foc_handle;

static void motorControl_adcCallback(void)
{
    /* ADC 注入转换完成后立即运行 FOC，保证采样、电流环和 PWM 更新同步。 */
    /* 编码器闭环：先刷新 AS5600/PLL，再用 PLL 角度做 Park 变换和 FOC 控制。 */
    encoder_update();

    // 更新 FOC 反馈结构体，供环路计算使用
    motor_foc_handle.feedback.angle_el = encoder_get_pllAngle() - motor_foc_handle.angle_offset; // 电角度 = PLL 估计角度 - 零点偏移
    motor_foc_handle.feedback.i_dq = park_transform(clark_transform(currentSense_get_injectedValue()), motor_foc_handle.feedback.angle_el); // 电流采样经过 Clarke/Park 变换得到 d/q 轴电流
    motor_foc_handle.feedback.speed_rpm = encoder_get_pllSpeed();
    motor_foc_handle.feedback.position_rad = encoder_get_mechanicalPosition();

    // 执行 FOC 控制步进，计算电压指令并更新 PWM 输出
    foc_step(&motor_foc_handle, FOC_SPEED_LOOP_DIVIDER, FOC_POSITION_LOOP_DIVIDER);
}

void motorControl_init(foc_mode_t mode)
{
    foc_init(&motor_foc_handle);
    foc_set_mode(&motor_foc_handle, mode);

    switch (mode)
    {
    case FOC_MODE_CURRENT:
        /* 电流模式：直接给定 id/iq，适合电流环整定和开环力矩测试。 */
        /* d/q 轴电流环共用一组 PI 参数，输出限幅对应允许的电压指令范围。 */
        pid_init(&motor_foc_handle.controller.id, PID_ROLE_CURRENT, CURRENT_PID_MODE, CURRENT_PID_KP, CURRENT_PID_KI, CURRENT_PID_KD, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX);
        pid_init(&motor_foc_handle.controller.iq, PID_ROLE_CURRENT, CURRENT_PID_MODE, CURRENT_PID_KP, CURRENT_PID_KI, CURRENT_PID_KD, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX);
        foc_alignment_zero(&motor_foc_handle);
        break;

    case FOC_MODE_SPEED:
        /* 速度模式：速度 PI 输出 iq，id 固定为 0。 */
        pid_init(&motor_foc_handle.controller.id, PID_ROLE_CURRENT, CURRENT_PID_MODE, CURRENT_PID_KP, CURRENT_PID_KI, CURRENT_PID_KD, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX);
        pid_init(&motor_foc_handle.controller.iq, PID_ROLE_CURRENT, CURRENT_PID_MODE, CURRENT_PID_KP, CURRENT_PID_KI, CURRENT_PID_KD, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX);
        /* 速度环输出为目标 q 轴电流，因此限幅等效于转矩电流限幅。 */
        pid_init(&motor_foc_handle.controller.speed, PID_ROLE_SPEED, SPEED_PID_MODE, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);
        foc_alignment_zero(&motor_foc_handle);
        break;

    case FOC_MODE_POSITION:
        /* 位置模式：位置 PD/PI -> 电流环，启动时将当前位置归零。 */
        pid_init(&motor_foc_handle.controller.id, PID_ROLE_CURRENT, CURRENT_PID_MODE, CURRENT_PID_KP, CURRENT_PID_KI, CURRENT_PID_KD, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX);
        pid_init(&motor_foc_handle.controller.iq, PID_ROLE_CURRENT, CURRENT_PID_MODE, CURRENT_PID_KP, CURRENT_PID_KI, CURRENT_PID_KD, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX);
        /* 位置环直接输出目标 q 轴电流，用作位置 PD/PI 与电流环级联。 */
        pid_init(&motor_foc_handle.controller.position, PID_ROLE_POSITION, POSITION_PID_MODE, POSITION_PID_KP, POSITION_PID_KI, POSITION_PID_KD, POSITION_PID_OUT_MIN, POSITION_PID_OUT_MAX);
        foc_alignment_zero(&motor_foc_handle);
        encoder_update();
        encoder_reset_mechanicalPosition(0.0f);
        foc_set_currentTarget(&motor_foc_handle, 0.0f, 0.0f);
        foc_set_speedTarget(&motor_foc_handle, 0.0f);
        break;

    default:
        break;
    }

    /* 注册实时控制回调。后续每次 ADC 注入采样完成都会进入 motorControl_adcCallback() */
    adc_register_injectedCallback(motorControl_adcCallback);
}

void motorControl_debugPrint(void)
{
    /* 各模式发送的数据通道数不同，应与 VOFA 上位机曲线配置保持一致。 */
    switch (motor_foc_handle.cmd.mode)
    {
    case FOC_MODE_CURRENT:
    {
        float data[10] = {motor_foc_handle.feedback.i_dq.d, motor_foc_handle.feedback.i_dq.q, motor_foc_handle.feedback.speed_rpm,
                          motor_foc_handle.feedback.angle_el,
                          motor_foc_handle.state.v_d_pi, motor_foc_handle.state.v_q_pi, motor_foc_handle.state.v_d_ff,
                          motor_foc_handle.state.v_q_ff, motor_foc_handle.state.v_d_out, motor_foc_handle.state.v_q_out};
        vofa_send(data, 10);
        break;
    }

    case FOC_MODE_SPEED:
    {
        float angle_deg = wrap_0_2pi(motor_foc_handle.feedback.angle_el) * 57.2958f;
        float pll_angle_deg = wrap_0_2pi(motor_foc_handle.feedback.angle_el) * 57.2958f;
        float data[14] = {motor_foc_handle.feedback.speed_rpm, angle_deg, pll_angle_deg, motor_foc_handle.feedback.i_dq.d, motor_foc_handle.feedback.i_dq.q,
                          motor_foc_handle.cmd.target_iq, motor_foc_handle.cmd.target_id, motor_foc_handle.state.v_d_pi, motor_foc_handle.state.v_q_pi,
                          motor_foc_handle.state.v_d_ff, motor_foc_handle.state.v_q_ff, motor_foc_handle.state.v_d_out,
                          motor_foc_handle.state.v_q_out,
                          sqrtf(motor_foc_handle.state.v_d_out * motor_foc_handle.state.v_d_out + motor_foc_handle.state.v_q_out * motor_foc_handle.state.v_q_out)};
        vofa_send(data, 14);
        break;
    }

    case FOC_MODE_POSITION:
    {
        float angle_deg = wrap_0_2pi(motor_foc_handle.feedback.angle_el) * 57.2958f;
        float pll_angle_deg = wrap_0_2pi(motor_foc_handle.feedback.angle_el) * 57.2958f;
        float data[18] = {motor_foc_handle.feedback.position_rad, motor_foc_handle.cmd.target_position, motor_foc_handle.cmd.target_position - motor_foc_handle.feedback.position_rad,
                          motor_foc_handle.feedback.speed_rpm, motor_foc_handle.cmd.target_speed, angle_deg, pll_angle_deg,
                          motor_foc_handle.feedback.i_dq.d, motor_foc_handle.feedback.i_dq.q,
                          motor_foc_handle.cmd.target_iq, motor_foc_handle.cmd.target_id, motor_foc_handle.state.v_d_pi, motor_foc_handle.state.v_q_pi,
                          motor_foc_handle.state.v_d_ff, motor_foc_handle.state.v_q_ff, motor_foc_handle.state.v_d_out,
                          motor_foc_handle.state.v_q_out,
                          sqrtf(motor_foc_handle.state.v_d_out * motor_foc_handle.state.v_d_out + motor_foc_handle.state.v_q_out * motor_foc_handle.state.v_q_out)};
        vofa_send(data, 18);
        break;
    }

    default:
        break;
    }
}

void motorControl_setMode(foc_mode_t mode)
{
    foc_set_mode(&motor_foc_handle, mode);
}

void motorControl_setSpeed(float rpm)
{
    foc_set_speedTarget(&motor_foc_handle, rpm);
}

void motorControl_setCurrent(float target_id, float target_iq)
{
    foc_set_currentTarget(&motor_foc_handle, target_id, target_iq);
}

void motorControl_setPositionRev(float position_rev)
{
    /* 位置单位：机械圈数，内部转换为机械弧度。 */
    foc_set_positionTarget(&motor_foc_handle, position_rev * MATH_TWO_PI);
}

void motorControl_resetPosition(float position_rad)
{
    /* 重置机械位置后同步清空外环积分和目标输出，防止位置突变造成冲击。 */
    encoder_reset_mechanicalPosition(position_rad);
    pid_reset(&motor_foc_handle.controller.position);
    pid_reset(&motor_foc_handle.controller.speed);
    foc_reset_motionTarget(&motor_foc_handle);
}
