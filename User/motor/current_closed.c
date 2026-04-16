#include "current_closed.h"

static foc_t foc_current_closed_handle;

static pid_controller_t pid_id;
static pid_controller_t pid_iq;

// 打印用
static dq_t i_dq_temp = {
    .d = 0.0f,
    .q = 0.0f,
};

static float speed_temp = 0.0f;
static float angle_el_temp = 0.0f;
static float pll_angle_el_temp = 0.0f;
static float adc_inj_irq_hz_temp = 0.0f;
static float adc_inj_callback_hz_temp = 0.0f;
static float v_d_pi_temp = 0.0f;
static float v_q_pi_temp = 0.0f;
static float v_d_ff_temp = 0.0f;
static float v_q_ff_temp = 0.0f;
static float v_d_out_temp = 0.0f;
static float v_q_out_temp = 0.0f;

// 电流闭环模式回调
static void current_closed_callback(void)
{
    encoder_update();

    // 控制使用PLL估计角度；原始角度只用于调试观察
    float angle_el = angle_wrap_0_2pi(encoder_get_angle() - foc_current_closed_handle.angle_offset);
    float angle_raw = angle_wrap_0_2pi(encoder_get_encoder_angle() - foc_current_closed_handle.angle_offset);

    // 获取电流反馈值
    adc_values_t adc_values;
    adc_get_injected_values(&adc_values);

    // Clark 变换
    abc_t i_abc = {.a = adc_values.ia, .b = adc_values.ib, .c = adc_values.ic};
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // Park 变换
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 打印用
    i_dq_temp = i_dq;
    speed_temp = encoder_get_speed();
    angle_el_temp = angle_raw;
    pll_angle_el_temp = angle_el;

    // 电流闭环
    foc_current_loop_run(&foc_current_closed_handle, i_dq, angle_el);

    // 打印用 Ud/Uq 及其 PI/前馈分量
    v_d_pi_temp = foc_current_closed_handle.v_d_pi;
    v_q_pi_temp = foc_current_closed_handle.v_q_pi;
    v_d_ff_temp = foc_current_closed_handle.v_d_ff;
    v_q_ff_temp = foc_current_closed_handle.v_q_ff;
    v_d_out_temp = foc_current_closed_handle.v_d_out;
    v_q_out_temp = foc_current_closed_handle.v_q_out;
}

void current_closed_init(float id, float iq)
{
    // 初始化电流环 PID 控制器
    pid_init(&pid_id, PID_TYPE_CURRENT, 5.02f, 0.267f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_iq, PID_TYPE_CURRENT, 5.02f, 0.267f, -U_DC / 2.0f, U_DC / 2.0f);

    // 初始化 FOC 控制句柄
    foc_init(&foc_current_closed_handle, &pid_id, &pid_iq, NULL);

    // 设置目标电流
    foc_set_target_id(&foc_current_closed_handle, id);
    foc_set_target_iq(&foc_current_closed_handle, iq);

    // 零点对齐
    foc_alignment(&foc_current_closed_handle);

    // 注册回调函数
    adc_register_injected_callback(current_closed_callback);
}

void print_current_info(void)
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

    float data[13] = {i_dq_temp.d, i_dq_temp.q, speed_temp, angle_el_temp, pll_angle_el_temp, adc_inj_irq_hz_temp, adc_inj_callback_hz_temp,
                      v_d_pi_temp, v_q_pi_temp, v_d_ff_temp, v_q_ff_temp, v_d_out_temp, v_q_out_temp};
    printf_vofa(data, 13);
}
