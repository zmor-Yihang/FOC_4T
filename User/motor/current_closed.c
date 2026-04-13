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
static float adc_inj_irq_cnt_temp = 0.0f;

// 电流闭环模式回调
static void current_closed_callback(void)
{
    // 计算角度
    float angle_el = encoder_get_angle_rad() - foc_current_closed_handle.angle_offset;

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

    encoder_update();
    speed_temp = encoder_get_speed_rpm();
    adc_inj_irq_cnt_temp = (float)adc_get_injected_irq_count();

    // 电流闭环
    foc_current_loop_run(&foc_current_closed_handle, i_dq, angle_el);
}

void current_closed_init(float id, float iq)
{
    // 初始化电流环 PID 控制器
    pid_init(&pid_id, PID_TYPE_CURRENT, 1.0f, 1.97f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_iq, PID_TYPE_CURRENT, 1.0f, 1.97f, -U_DC / 2.0f, U_DC / 2.0f);
    
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
    float data[4] = {i_dq_temp.d, i_dq_temp.q, speed_temp, adc_inj_irq_cnt_temp};
    printf_vofa(data, 4);
}