#include "motor_control.h"

/*
 * 电机控制总入口：
 * 1. 根据 FOC 模式选择电流环、速度环或位置环闭环。
 * 2. ADC 注入转换完成中断作为 FOC 实时控制节拍。
 * 3. 编码器 / 观测器提供电角度与速度反馈，foc_step() 完成环路计算与 SVPWM 输出。
 */
static foc_t motor_foc_handle;
static foc_mode_t motor_foc_mode = FOC_MODE_CURRENT;
static abc_t motor_debug_i_abc;

static void motorControl_adcCallback(void)
{
    /* ADC 注入转换完成后立即运行 FOC，保证采样、电流环和 PWM 更新同步。 */
    /* 编码器闭环：先刷新 AS5600/PLL，再用 PLL 角度做 Park 变换和 FOC 控制。 */
    encoder_update();

    uint8_t speed_divider = 1U;
    uint8_t position_divider = 1U;

    // 获取当前电流采样值和电角度
    abc_t i_abc = currentSense_get_injectedValue();
    float angle_ctrl = encoder_get_pllAngle() - motor_foc_handle.angle_offset;

    // 坐标变换：三相电流 -> alpha/beta -> d/q
    alphabeta_t i_alphabeta = clark_transform(i_abc);
    dq_t i_dq = park_transform(i_alphabeta, angle_ctrl);

    // 更新 FOC 反馈结构体，供环路计算和调试观察使用
    motor_foc_handle.feedback.i_dq = i_dq;
    motor_foc_handle.feedback.angle_el = angle_ctrl;
    motor_foc_handle.feedback.speed_rpm = encoder_get_pllSpeed();

    /* 仅位置模式需要机械位置反馈；电流/速度模式只依赖电角度与转速反馈。 */
    if (motor_foc_mode == FOC_MODE_POSITION)
    {
        motor_foc_handle.feedback.position_rad = encoder_get_mechanicalPosition();
    }

    /* 速度环属于外环，只在速度模式和位置模式下启用，并按分频系数降频执行。 */
    if ((motor_foc_mode == FOC_MODE_SPEED) || (motor_foc_mode == FOC_MODE_POSITION))
    {
        speed_divider = FOC_SPEED_LOOP_DIVIDER;
    }

    /* 位置环只在位置模式下启用，并按更外层的分频系数降频执行。 */
    if (motor_foc_mode == FOC_MODE_POSITION)
    {
        position_divider = FOC_POSITION_LOOP_DIVIDER;
    }

    // 执行 FOC 控制步进，计算电压指令并更新 PWM 输出
    foc_step(&motor_foc_handle, speed_divider, position_divider);

    /* 中断内仅缓存三相原始电流，串口发送放到主循环。 */
    motor_debug_i_abc = i_abc;
}

void motorControl_init(foc_mode_t mode, const foc_cmd_t *cmd)
{
    motor_foc_mode = mode;
    foc_init(&motor_foc_handle);
    motor_foc_handle.cmd.mode = mode;

    /* 清空调试缓存，避免切换模式后串口输出残留旧状态。 */
    motor_debug_i_abc = (abc_t){0};

    switch (mode)
    {
    case FOC_MODE_CURRENT:
        /* 电流模式：直接给定 id/iq，适合电流环整定和开环力矩测试。 */
        /* d/q 轴电流环共用一组 PI 参数，输出限幅对应允许的电压指令范围。 */
        pid_init(&motor_foc_handle.controller.id, PID_ROLE_CURRENT, CURRENT_PID_MODE, CURRENT_PID_KP, CURRENT_PID_KI, CURRENT_PID_KD, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX);
        pid_init(&motor_foc_handle.controller.iq, PID_ROLE_CURRENT, CURRENT_PID_MODE, CURRENT_PID_KP, CURRENT_PID_KI, CURRENT_PID_KD, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX);
        motor_foc_handle.cmd.target_id = (cmd != NULL) ? cmd->target_id : 0.0f;
        motor_foc_handle.cmd.target_iq = (cmd != NULL) ? cmd->target_iq : 0.0f;
        foc_alignment_zero(&motor_foc_handle);
        break;

    case FOC_MODE_SPEED:
        /* 速度模式：速度 PI 输出 iq，id 固定为 0。 */
        pid_init(&motor_foc_handle.controller.id, PID_ROLE_CURRENT, CURRENT_PID_MODE, CURRENT_PID_KP, CURRENT_PID_KI, CURRENT_PID_KD, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX);
        pid_init(&motor_foc_handle.controller.iq, PID_ROLE_CURRENT, CURRENT_PID_MODE, CURRENT_PID_KP, CURRENT_PID_KI, CURRENT_PID_KD, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX);
        /* 速度环输出为目标 q 轴电流，因此限幅等效于转矩电流限幅。 */
        pid_init(&motor_foc_handle.controller.speed, PID_ROLE_SPEED, SPEED_PID_MODE, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);
        motor_foc_handle.cmd.target_id = 0.0f;
        motor_foc_handle.cmd.target_speed = (cmd != NULL) ? cmd->target_speed : 0.0f;
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
        motor_foc_handle.cmd.target_id = 0.0f;
        motor_foc_handle.cmd.target_speed = 0.0f;
        motor_foc_handle.cmd.target_iq = 0.0f;
        motor_foc_handle.cmd.target_position = (cmd != NULL) ? cmd->target_position : 0.0f;
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
    switch (motor_foc_mode)
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
        float data[17] = {motor_foc_handle.feedback.speed_rpm, angle_deg, pll_angle_deg, motor_foc_handle.feedback.i_dq.d, motor_foc_handle.feedback.i_dq.q,
                          motor_debug_i_abc.a, motor_debug_i_abc.b, motor_debug_i_abc.c,
                          motor_foc_handle.cmd.target_iq, motor_foc_handle.cmd.target_id, motor_foc_handle.state.v_d_pi, motor_foc_handle.state.v_q_pi,
                          motor_foc_handle.state.v_d_ff, motor_foc_handle.state.v_q_ff, motor_foc_handle.state.v_d_out,
                          motor_foc_handle.state.v_q_out,
                          sqrtf(motor_foc_handle.state.v_d_out * motor_foc_handle.state.v_d_out + motor_foc_handle.state.v_q_out * motor_foc_handle.state.v_q_out)};
        vofa_send(data, 17);
        break;
    }

    case FOC_MODE_POSITION:
    {
        float angle_deg = wrap_0_2pi(motor_foc_handle.feedback.angle_el) * 57.2958f;
        float pll_angle_deg = wrap_0_2pi(motor_foc_handle.feedback.angle_el) * 57.2958f;
        float data[21] = {motor_foc_handle.feedback.position_rad, motor_foc_handle.cmd.target_position, motor_foc_handle.cmd.target_position - motor_foc_handle.feedback.position_rad,
                          motor_foc_handle.feedback.speed_rpm, motor_foc_handle.cmd.target_speed, angle_deg, pll_angle_deg,
                          motor_foc_handle.feedback.i_dq.d, motor_foc_handle.feedback.i_dq.q, motor_debug_i_abc.a, motor_debug_i_abc.b, motor_debug_i_abc.c,
                          motor_foc_handle.cmd.target_iq, motor_foc_handle.cmd.target_id, motor_foc_handle.state.v_d_pi, motor_foc_handle.state.v_q_pi,
                          motor_foc_handle.state.v_d_ff, motor_foc_handle.state.v_q_ff, motor_foc_handle.state.v_d_out,
                          motor_foc_handle.state.v_q_out,
                          sqrtf(motor_foc_handle.state.v_d_out * motor_foc_handle.state.v_d_out + motor_foc_handle.state.v_q_out * motor_foc_handle.state.v_q_out)};
        vofa_send(data, 21);
        break;
    }

    default:
        break;
    }
}

void motorControl_setPosition(float position_rad)
{
    /* 位置单位：机械弧度。 */
    motor_foc_handle.cmd.target_position = position_rad;
}

void motorControl_setPositionRev(float position_rev)
{
    /* 位置单位：机械圈数，内部转换为机械弧度。 */
    motor_foc_handle.cmd.target_position = position_rev * MATH_TWO_PI;
}

void motorControl_resetPosition(float position_rad)
{
    /* 重置机械位置后同步清空外环积分和目标输出，防止位置突变造成冲击。 */
    encoder_reset_mechanicalPosition(position_rad);
    pid_reset(&motor_foc_handle.controller.position);
    pid_reset(&motor_foc_handle.controller.speed);
    motor_foc_handle.cmd.target_speed = 0.0f;
    motor_foc_handle.cmd.target_iq = 0.0f;
}

foc_t *motorControl_getFocHandle(void)
{
    return &motor_foc_handle;
}

foc_mode_t motorControl_getMode(void)
{
    return motor_foc_mode;
}
