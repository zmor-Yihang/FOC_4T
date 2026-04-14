#include "speed_closed.h"

static foc_t foc_speed_closed_handle;

static pid_controller_t pid_id;
static pid_controller_t pid_iq;

static pid_controller_t pid_speed;

static uint8_t speed_loop_divider = 10;

// 打印用
static float speed_rpm_temp = 0.0f;
static float angle_el_temp = 0.0f;
static float pll_angle_el_temp = 0.0f;
static float id_temp = 0.0f;
static float iq_temp = 0.0f;
static float ia_temp = 0.0f;
static float ib_temp = 0.0f;
static float ic_temp = 0.0f;
static float adc_inj_irq_hz_temp = 0.0f;
static float adc_inj_callback_hz_temp = 0.0f;
static float target_iq_temp = 0.0f;
static float v_d_out_temp = 0.0f;
static float v_q_out_temp = 0.0f;
static float v_mag_temp = 0.0f;

static void speed_closed_callback(void)
{
    // 更新速度
    encoder_update();

    // 控制使用PLL估计角度；原始角度只用于调试观察
    float angle_raw = encoder_get_angle_rad() - foc_speed_closed_handle.angle_offset;
    float angle_el = encoder_get_pll_angle_rad() - foc_speed_closed_handle.angle_offset;
    float speed_feedback = encoder_get_speed_rpm();

    // 获取电流反馈值
    adc_values_t adc_values;
    adc_get_injected_values(&adc_values);

    // Clark 变换
    abc_t i_abc = {.a = adc_values.ia, .b = adc_values.ib, .c = adc_values.ic};
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // Park 变换
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 保存电流值用于打印
    id_temp = i_dq.d;
    iq_temp = i_dq.q;
    ia_temp = i_abc.a;
    ib_temp = i_abc.b;
    ic_temp = i_abc.c;
    speed_rpm_temp = speed_feedback;
    angle_el_temp = angle_raw;
    pll_angle_el_temp = angle_el;

    // 速度闭环
    foc_speed_loop_run(&foc_speed_closed_handle, i_dq, angle_el, speed_feedback, speed_loop_divider);

    target_iq_temp = foc_speed_closed_handle.target_iq;
    v_d_out_temp = foc_speed_closed_handle.v_d_out;
    v_q_out_temp = foc_speed_closed_handle.v_q_out;
    v_mag_temp = sqrtf(v_d_out_temp * v_d_out_temp + v_q_out_temp * v_q_out_temp);
}

void speed_closed_init(float speed_rpm)
{
    // 初始化速度环 PID 控制器
    pid_init(&pid_id, PID_TYPE_CURRENT, 8.1f, 0.198f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_iq, PID_TYPE_CURRENT, 8.1f, 0.198f, -U_DC / 2.0f, U_DC / 2.0f);

    pid_init(&pid_speed, PID_TYPE_SPEED, 0.0082f, 0.001f, -1.5f, 1.5f);

    // 初始化 FOC 控制句柄
    foc_init(&foc_speed_closed_handle, &pid_id, &pid_iq, &pid_speed);

    // 设置目标速度
    foc_set_target_id(&foc_speed_closed_handle, 0.0f);
    foc_set_target_speed(&foc_speed_closed_handle, speed_rpm);

    // 零点对齐
    foc_alignment(&foc_speed_closed_handle);

    // 注册回调函数
    adc_register_injected_callback(speed_closed_callback);
}

void print_speed_info(void)
{
    static uint32_t last_tick_ms = 0U;
    static uint32_t last_irq_count = 0U;
    static uint32_t last_callback_count = 0U;

    uint32_t now_tick_ms = HAL_GetTick();
    if (last_tick_ms == 0U)
    {
        last_tick_ms = now_tick_ms;
        last_irq_count = adc_get_injected_irq_count();
        last_callback_count = adc_get_injected_callback_count();
    }
    else
    {
        uint32_t delta_ms = now_tick_ms - last_tick_ms;
        if (delta_ms >= 100U)
        {
            uint32_t irq_count = adc_get_injected_irq_count();
            uint32_t callback_count = adc_get_injected_callback_count();

            adc_inj_irq_hz_temp = ((float)(irq_count - last_irq_count) * 1000.0f) / (float)delta_ms;
            adc_inj_callback_hz_temp = ((float)(callback_count - last_callback_count) * 1000.0f) / (float)delta_ms;

            last_irq_count = irq_count;
            last_callback_count = callback_count;
            last_tick_ms = now_tick_ms;
        }
    }

    // 归一化角度到 [0, 2π) 范围
    float angle_normalized = fmodf(angle_el_temp, 2.0f * M_PI);
    if (angle_normalized < 0.0f)
    {
        angle_normalized += 2.0f * M_PI;
    }
    // 转换为角度 (0-360°)
    float angle_deg = angle_normalized * 57.2958f;

    // PLL估计电角度也归一化并转换到角度制
    float pll_angle_normalized = fmodf(pll_angle_el_temp, 2.0f * M_PI);
    if (pll_angle_normalized < 0.0f)
    {
        pll_angle_normalized += 2.0f * M_PI;
    }
    float pll_angle_deg = pll_angle_normalized * 57.2958f;

    float data[14] = {speed_rpm_temp, angle_deg, pll_angle_deg, id_temp, iq_temp, ia_temp, ib_temp, ic_temp,
                      adc_inj_irq_hz_temp, adc_inj_callback_hz_temp, target_iq_temp, v_d_out_temp, v_q_out_temp, v_mag_temp};
    printf_vofa(data, 14);
}