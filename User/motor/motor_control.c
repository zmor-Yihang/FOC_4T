#include "motor_control.h"

/*
 * 电机控制总入口：
 * 1. 根据 motor_ctrl_mode 选择电流环、速度环、位置环、弱磁或磁链观测器闭环。
 * 2. ADC 注入转换完成中断作为 FOC 实时控制节拍。
 * 3. 编码器 / 观测器提供电角度与速度反馈，foc_step() 完成环路计算与 SVPWM 输出。
 */
static foc_t motor_foc_handle;
static motor_ctrl_mode_t motor_ctrl_mode = MOTOR_CTRL_MODE_CURRENT;
static fluxobserver_t motor_flux_observer;

/* 磁链观测器参数：由电机参数、控制周期和 PLL 参数共同决定观测器动态。 */
static const fluxobserver_cfg_t motor_flux_observer_cfg = {
    .rs = MOTOR_RS_Ω,
    .ls = 0.5f * (MOTOR_LD_H + MOTOR_LQ_H),
    .psi_m = MOTOR_PSI_F,
    .poles = MOTOR_POLE_PAIRS,
    .ts = FLUX_OBSERVER_TS_S,
    .gamma = FLUX_OBSERVER_GAMMA,
    .pll_kp = FLUX_OBSERVER_PLL_KP,
    .pll_ki = FLUX_OBSERVER_PLL_KI,
    .pll_speed_limit_rpm = FLUX_OBSERVER_PLL_SPEED_LIMIT_RPM,
};

/* VOFA 调试缓存：只在控制中断中更新，在低频 debugPrint 中发送，避免打印影响实时环。 */
typedef struct
{
    abc_t i_abc;
    dq_t i_dq;
    float speed_rpm;
    float observer_speed_rpm;
    float angle_meas;
    float angle_ctrl;
    float position_rad;
    float target_position_rad;
    float position_error_rad;
    float target_speed_rpm;
    float target_id;
    float target_iq;
    float v_mag;
    float flux_angle;
    float flux_linkage;
    float i2c_read_state;
    float adc_inj_irq_hz;
    float adc_inj_callback_hz;
} motor_ctrl_debug_t;

static motor_ctrl_debug_t motor_debug;

static void motorControl_initCurrentPid(void)
{
    /* d/q 轴电流环共用一组 PI 参数，输出限幅对应允许的电压指令范围。 */
    pid_init(&motor_foc_handle.controller.id, PID_TYPE_CURRENT, CURRENT_PID_KP, CURRENT_PID_KI, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX);
    pid_init(&motor_foc_handle.controller.iq, PID_TYPE_CURRENT, CURRENT_PID_KP, CURRENT_PID_KI, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX);
}

static void motorControl_initSpeedPid(void)
{
    /* 速度环输出为目标 q 轴电流，因此限幅等效于转矩电流限幅。 */
    pid_init(&motor_foc_handle.controller.speed, PID_TYPE_SPEED, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);
}

static void motorControl_initPositionPid(void)
{
    /* 位置环输出为目标速度，速度环再输出目标 q 轴电流。 */
    pid_init(&motor_foc_handle.controller.position, PID_TYPE_POSITION, POSITION_PID_KP, POSITION_PID_KI, POSITION_PID_OUT_MIN, POSITION_PID_OUT_MAX);
}

static void motorControl_initFluxObserverPid(void)
{
    /* 磁链观测器模式使用独立调试参数，便于和编码器闭环参数分开整定。 */
    pid_init(&motor_foc_handle.controller.id, PID_TYPE_CURRENT, 2.02f, 2670.0f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&motor_foc_handle.controller.iq, PID_TYPE_CURRENT, 2.02f, 2670.0f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&motor_foc_handle.controller.speed, PID_TYPE_SPEED, 0.004f, 2.5f, -1.0f, 1.0f);
}

static void motorControl_saveFocDebug(void)
{
    /* 统一保存 foc_step() 之后的目标值和最终输出电压幅值，供各模式调试输出复用。 */
    motor_debug.target_id = motor_foc_handle.cmd.target_id;
    motor_debug.target_iq = motor_foc_handle.cmd.target_iq;
    motor_debug.target_speed_rpm = motor_foc_handle.cmd.target_speed;
    motor_debug.target_position_rad = motor_foc_handle.cmd.target_position;
    motor_debug.v_mag = sqrtf(motor_foc_handle.state.v_d_out * motor_foc_handle.state.v_d_out + motor_foc_handle.state.v_q_out * motor_foc_handle.state.v_q_out);
}

static void motorControl_encoderStep(void)
{
    /* 编码器闭环：先刷新 AS5600/PLL，再用 PLL 角度做 Park 变换和 FOC 控制。 */
    encoder_update();

    float angle_ctrl = encoder_get_pllAngle() - motor_foc_handle.angle_offset;
    float angle_meas = encoder_get_encoderAngle() - motor_foc_handle.angle_offset;
    float speed_feedback = encoder_get_pllSpeed();

    abc_t i_abc;
    currentSense_get_injectedValue(&i_abc);

    alphabeta_t i_alphabeta = clark_transform(i_abc);
    dq_t i_dq = park_transform(i_alphabeta, angle_ctrl);

    motor_foc_handle.feedback.i_dq = i_dq;
    motor_foc_handle.feedback.angle_el = angle_ctrl;
    motor_foc_handle.feedback.speed_rpm = speed_feedback;

    /* 只有位置模式需要机械位置反馈，避免其他模式额外处理位置累计量。 */
    if (motor_ctrl_mode == MOTOR_CTRL_MODE_POSITION)
    {
        motor_foc_handle.feedback.position_rad = encoder_get_mechanicalPosition();
    }

    uint8_t speed_divider = 1U;
    uint8_t position_divider = 1U;

    /* 电流环每个 ADC 周期运行；速度环/位置环按分频运行，降低外环带宽。 */
    if ((motor_ctrl_mode == MOTOR_CTRL_MODE_SPEED) ||
        (motor_ctrl_mode == MOTOR_CTRL_MODE_SPEED_WEAK) ||
        (motor_ctrl_mode == MOTOR_CTRL_MODE_POSITION))
    {
        speed_divider = FOC_SPEED_LOOP_DIVIDER;
    }

    if (motor_ctrl_mode == MOTOR_CTRL_MODE_POSITION)
    {
        position_divider = FOC_POSITION_LOOP_DIVIDER;
    }

    foc_step(&motor_foc_handle, speed_divider, position_divider);

    /* 中断内仅缓存数据，串口发送放到主循环的 motorControl_debugPrint()。 */
    motor_debug.i_abc = i_abc;
    motor_debug.i_dq = i_dq;
    motor_debug.speed_rpm = speed_feedback;
    motor_debug.angle_meas = angle_meas;
    motor_debug.angle_ctrl = angle_ctrl;
    motor_debug.position_rad = motor_foc_handle.feedback.position_rad;
    motor_debug.position_error_rad = motor_foc_handle.cmd.target_position - motor_foc_handle.feedback.position_rad;
    motor_debug.i2c_read_state = (float)i2c_get_readState();
    motorControl_saveFocDebug();
}

static void motorControl_fluxObserverStep(void)
{
    /* 观测器闭环：编码器仍用于对比调试，控制角度来自磁链观测器 PLL。 */
    encoder_update();

    abc_t i_abc;
    currentSense_get_injectedValue(&i_abc);

    alphabeta_t i_alphabeta = clark_transform(i_abc);

    motor_flux_observer.i_alpha = i_alphabeta.alpha;
    motor_flux_observer.i_beta = i_alphabeta.beta;
    /* 使用上一拍输出电压和当前电流估算磁链角度。 */
    fluxObserver_estimate(&motor_flux_observer);

    float angle_encoder = wrap_pm_pi(encoder_get_pllAngle() - motor_foc_handle.angle_offset);
    float speed_encoder = encoder_get_pllSpeed();
    float angle_ctrl = wrap_pm_pi(fluxObserver_get_angle(&motor_flux_observer) - motor_foc_handle.angle_offset);
    float speed_observer = fluxObserver_get_speed(&motor_flux_observer);

    dq_t i_dq = park_transform(i_alphabeta, angle_ctrl);

    motor_foc_handle.feedback.i_dq = i_dq;
    motor_foc_handle.feedback.angle_el = angle_ctrl;
    motor_foc_handle.feedback.speed_rpm = speed_observer;
    foc_step(&motor_foc_handle, FOC_SPEED_LOOP_DIVIDER, 1U);

    /* 将本拍 FOC 输出电压变换回 alpha/beta，作为下一拍磁链观测器输入。 */
    alphabeta_t u_alphabeta = ipark_transform((dq_t){.d = motor_foc_handle.state.v_d_out, .q = motor_foc_handle.state.v_q_out}, angle_ctrl);
    motor_flux_observer.u_alpha = u_alphabeta.alpha;
    motor_flux_observer.u_beta = u_alphabeta.beta;

    motor_debug.i_abc = i_abc;
    motor_debug.i_dq = i_dq;
    motor_debug.speed_rpm = speed_encoder;
    motor_debug.observer_speed_rpm = speed_observer;
    motor_debug.angle_meas = angle_encoder;
    motor_debug.angle_ctrl = angle_ctrl;
    motor_debug.flux_angle = wrap_pm_pi(motor_flux_observer.theta_est - motor_foc_handle.angle_offset);
    motor_debug.flux_linkage = sqrtf(motor_flux_observer.xhat_alpha * motor_flux_observer.xhat_alpha + motor_flux_observer.xhat_beta * motor_flux_observer.xhat_beta);
    motorControl_saveFocDebug();
}

static void motorControl_adcCallback(void)
{
    /* ADC 注入转换完成后立即运行 FOC，保证采样、电流环和 PWM 更新同步。 */
    if (motor_ctrl_mode == MOTOR_CTRL_MODE_FLUX_OBSERVER)
    {
        motorControl_fluxObserverStep();
    }
    else
    {
        motorControl_encoderStep();
    }
}

void motorControl_init(motor_ctrl_mode_t mode, const motor_ctrl_cmd_t *cmd)
{
    motor_ctrl_mode = mode;
    foc_init(&motor_foc_handle);

    /* 清空调试缓存，避免切换模式后串口输出残留旧状态。 */
    motor_debug = (motor_ctrl_debug_t){0};

    switch (mode)
    {
    case MOTOR_CTRL_MODE_CURRENT:
        /* 电流模式：直接给定 id/iq，适合电流环整定和开环力矩测试。 */
        foc_set_mode(&motor_foc_handle, FOC_MODE_CURRENT);
        motorControl_initCurrentPid();
        foc_set_id(&motor_foc_handle, (cmd != NULL) ? cmd->id : 0.0f);
        foc_set_iq(&motor_foc_handle, (cmd != NULL) ? cmd->iq : 0.0f);
        foc_alignment_zero(&motor_foc_handle);
        break;

    case MOTOR_CTRL_MODE_SPEED:
        /* 速度模式：速度 PI 输出 iq，id 固定为 0。 */
        foc_set_mode(&motor_foc_handle, FOC_MODE_SPEED);
        motorControl_initCurrentPid();
        motorControl_initSpeedPid();
        foc_set_id(&motor_foc_handle, 0.0f);
        foc_set_speed(&motor_foc_handle, (cmd != NULL) ? cmd->speed_rpm : 0.0f);
        foc_alignment_zero(&motor_foc_handle);
        break;

    case MOTOR_CTRL_MODE_POSITION:
        /* 位置模式：位置环 -> 速度环 -> 电流环，启动时将当前位置归零。 */
        foc_set_mode(&motor_foc_handle, FOC_MODE_POSITION);
        motorControl_initCurrentPid();
        motorControl_initSpeedPid();
        motorControl_initPositionPid();
        foc_alignment_zero(&motor_foc_handle);
        encoder_update();
        encoder_reset_mechanicalPosition(0.0f);
        foc_set_id(&motor_foc_handle, 0.0f);
        foc_set_speed(&motor_foc_handle, 0.0f);
        foc_set_position(&motor_foc_handle, (cmd != NULL) ? cmd->position_rad : 0.0f);
        break;

    case MOTOR_CTRL_MODE_SPEED_WEAK:
        /* 弱磁速度模式：弱磁算法由 FLUX_WEAK_ENABLE 宏在 loop_control.c 中统一开关。 */
        foc_set_mode(&motor_foc_handle, FOC_MODE_SPEED);
        motorControl_initCurrentPid();
        motorControl_initSpeedPid();
        foc_set_id(&motor_foc_handle, 0.0f);
        foc_set_speed(&motor_foc_handle, (cmd != NULL) ? cmd->speed_rpm : 0.0f);
        foc_alignment_zero(&motor_foc_handle);
        break;

    case MOTOR_CTRL_MODE_FLUX_OBSERVER:
        /* 磁链观测器模式：用观测角度闭环，不做零点对齐，主要用于无感算法调试。 */
        foc_set_mode(&motor_foc_handle, FOC_MODE_SPEED);
        motorControl_initFluxObserverPid();
        foc_set_id(&motor_foc_handle, 0.0f);
        foc_set_speed(&motor_foc_handle, (cmd != NULL) ? cmd->speed_rpm : 0.0f);
        fluxObserver_init(&motor_flux_observer, &motor_flux_observer_cfg);
        motor_flux_observer.u_alpha = 0.0f;
        motor_flux_observer.u_beta = 0.0f;
        break;

    default:
        break;
    }

    /* 注册实时控制回调。后续每次 ADC 注入采样完成都会进入 motorControl_adcCallback()。 */
    adc_register_injectedCallback(motorControl_adcCallback);
}

void motorControl_debugPrint(void)
{
    /* 各模式发送的数据通道数不同，应与 VOFA 上位机曲线配置保持一致。 */
    switch (motor_ctrl_mode)
    {
    case MOTOR_CTRL_MODE_CURRENT:
    {
        static uint32_t last_tick_ms = 0U;
        static uint32_t last_irq_count = 0U;
        static uint32_t last_callback_count = 0U;

        uint32_t now_tick_ms = HAL_GetTick();
        if (last_tick_ms == 0U)
        {
            last_tick_ms = now_tick_ms;
            last_irq_count = adcDebug_get_injectedIrqCount();
            last_callback_count = adcDebug_get_injectedCallbackCount();
        }
        else
        {
            /* 统计 ADC 注入中断和控制回调频率，用于确认控制周期是否稳定。 */
            uint32_t delta_ms = now_tick_ms - last_tick_ms;
            if (delta_ms >= 100U)
            {
                uint32_t irq_count = adcDebug_get_injectedIrqCount();
                uint32_t callback_count = adcDebug_get_injectedCallbackCount();

                motor_debug.adc_inj_irq_hz = ((float)(irq_count - last_irq_count) * 1000.0f) / (float)delta_ms;
                motor_debug.adc_inj_callback_hz = ((float)(callback_count - last_callback_count) * 1000.0f) / (float)delta_ms;

                last_irq_count = irq_count;
                last_callback_count = callback_count;
                last_tick_ms = now_tick_ms;
            }
        }

        float data[13] = {motor_debug.i_dq.d, motor_debug.i_dq.q, motor_debug.speed_rpm,
                          motor_debug.angle_meas, motor_debug.angle_ctrl, motor_debug.adc_inj_irq_hz, motor_debug.adc_inj_callback_hz,
                          motor_foc_handle.state.v_d_pi, motor_foc_handle.state.v_q_pi, motor_foc_handle.state.v_d_ff,
                          motor_foc_handle.state.v_q_ff, motor_foc_handle.state.v_d_out, motor_foc_handle.state.v_q_out};
        vofa_send(data, 13);
        break;
    }

    case MOTOR_CTRL_MODE_SPEED:
    {
        float angle_deg = wrap_0_2pi(motor_debug.angle_meas) * 57.2958f;
        float pll_angle_deg = wrap_0_2pi(motor_debug.angle_ctrl) * 57.2958f;
        float data[17] = {motor_debug.speed_rpm, angle_deg, pll_angle_deg, motor_debug.i_dq.d, motor_debug.i_dq.q,
                          motor_debug.i_abc.a, motor_debug.i_abc.b, motor_debug.i_abc.c,
                          motor_debug.target_iq, motor_debug.target_id, motor_foc_handle.state.v_d_pi, motor_foc_handle.state.v_q_pi,
                          motor_foc_handle.state.v_d_ff, motor_foc_handle.state.v_q_ff, motor_foc_handle.state.v_d_out,
                          motor_foc_handle.state.v_q_out, motor_debug.v_mag};
        vofa_send(data, 17);
        break;
    }

    case MOTOR_CTRL_MODE_SPEED_WEAK:
    {
        float angle_deg = wrap_0_2pi(motor_debug.angle_meas) * 57.2958f;
        float pll_angle_deg = wrap_0_2pi(motor_debug.angle_ctrl) * 57.2958f;
        float data[16] = {motor_debug.speed_rpm, angle_deg, pll_angle_deg, motor_debug.i_dq.d, motor_debug.i_dq.q,
                          motor_debug.i_abc.a, motor_debug.i_abc.b, motor_debug.i_abc.c,
                          motor_foc_handle.state.v_d_pi, motor_foc_handle.state.v_q_pi, motor_foc_handle.state.v_d_ff,
                          motor_foc_handle.state.v_q_ff, motor_foc_handle.state.v_d_out, motor_foc_handle.state.v_q_out,
                          motor_debug.v_mag, motor_debug.i2c_read_state};
        vofa_send(data, 16);
        break;
    }

    case MOTOR_CTRL_MODE_POSITION:
    {
        float angle_deg = wrap_0_2pi(motor_debug.angle_meas) * 57.2958f;
        float pll_angle_deg = wrap_0_2pi(motor_debug.angle_ctrl) * 57.2958f;
        float data[21] = {motor_debug.position_rad, motor_debug.target_position_rad, motor_debug.position_error_rad,
                          motor_debug.speed_rpm, motor_debug.target_speed_rpm, angle_deg, pll_angle_deg,
                          motor_debug.i_dq.d, motor_debug.i_dq.q, motor_debug.i_abc.a, motor_debug.i_abc.b, motor_debug.i_abc.c,
                          motor_debug.target_iq, motor_debug.target_id, motor_foc_handle.state.v_d_pi, motor_foc_handle.state.v_q_pi,
                          motor_foc_handle.state.v_d_ff, motor_foc_handle.state.v_q_ff, motor_foc_handle.state.v_d_out,
                          motor_foc_handle.state.v_q_out, motor_debug.v_mag};
        vofa_send(data, 21);
        break;
    }

    case MOTOR_CTRL_MODE_FLUX_OBSERVER:
    {
        float angle_encoder_deg = wrap_0_2pi(motor_debug.angle_meas) * 57.2958f;
        float angle_observer_deg = wrap_0_2pi(motor_debug.angle_ctrl) * 57.2958f;
        float flux_angle_deg = wrap_0_2pi(motor_debug.flux_angle) * 57.2958f;
        float data[6] = {motor_debug.speed_rpm, motor_debug.observer_speed_rpm, angle_encoder_deg, angle_observer_deg, flux_angle_deg, motor_debug.flux_linkage};
        vofa_send(data, 6);
        break;
    }

    default:
        break;
    }
}

void motorControl_setPosition(float position_rad)
{
    /* 位置单位：机械弧度。 */
    foc_set_position(&motor_foc_handle, position_rad);
}

void motorControl_setPositionRev(float position_rev)
{
    /* 位置单位：机械圈数，内部转换为机械弧度。 */
    foc_set_position(&motor_foc_handle, position_rev * MATH_TWO_PI);
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

motor_ctrl_mode_t motorControl_getMode(void)
{
    return motor_ctrl_mode;
}
