#include "speed_closed.h"

static foc_t foc_speed_closed_handle;

static pid_controller_t pid_id;
static pid_controller_t pid_iq;

static pid_controller_t pid_speed;

static uint8_t speed_loop_divider = 20;

// 打印用
static float speed_rpm_temp = 0.0f;
static float angle_el_temp = 0.0f;
static float id_temp = 0.0f;
static float iq_temp = 0.0f;
static float ia_temp = 0.0f;
static float ib_temp = 0.0f;
static float ic_temp = 0.0f;

static void speed_closed_callback(void)
{
    // 更新速度
    encoder_update_speed();

    // 获取角度和速度
    float angle_el = encoder_get_angle_rad() - foc_speed_closed_handle.angle_offset;
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
    angle_el_temp = angle_el;

    // 速度闭环
    foc_speed_loop_run(&foc_speed_closed_handle, i_dq, angle_el, speed_feedback, speed_loop_divider);
}

void speed_closed_init(float speed_rpm)
{
    // 初始化速度环 PID 控制器
    pid_init(&pid_id, PID_TYPE_CURRENT, 2.7f, 0.148f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_iq, PID_TYPE_CURRENT, 2.7f, 0.148f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_speed, PID_TYPE_SPEED, 0.0001f, 0.00157f, -3.5f, 3.5f);

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
    // 归一化角度到 [0, 2π) 范围
    float angle_normalized = fmodf(angle_el_temp, 2.0f * M_PI);
    if (angle_normalized < 0.0f)
    {
        angle_normalized += 2.0f * M_PI;
    }
    // 转换为角度 (0-360°)
    float angle_deg = angle_normalized * 57.2958f;

    float data[7] = {speed_rpm_temp, angle_deg, id_temp, iq_temp, ia_temp, ib_temp, ic_temp};
    printf_vofa(data, 7);
}