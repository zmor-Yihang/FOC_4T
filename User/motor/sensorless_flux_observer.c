#include "sensorless_flux_observer.h"

// FOC 控制句柄
static foc_t foc_flux_handle;

// 非线性磁链观测器实例
static fluxobserver_t flux_observer;

// PID 控制器实例
static pid_controller_t pid_id;
static pid_controller_t pid_iq;
static pid_controller_t pid_speed;

// 控制参数
static const uint8_t SPEED_LOOP_DIV = 10;

// 调试打印变量
static float speed_rpm_actual_temp = 0.0f;
static float speed_rpm_observer_temp = 0.0f;
static float angle_el_actual_temp = 0.0f;
static float angle_el_observer_temp = 0.0f;

/**
 * @brief ADC 注入组中断回调
 * @note 运行流程：采样电流 -> 坐标变换 -> 使用观测器角度执行无感闭环 -> 更新观测器
 */
static void sensorless_flux_observer_callback(void)
{
    // 读取三相电流
    adc_values_t adc_values;
    adc1_get_injected_values(&adc_values);

    // Clark 变换，得到 αβ 轴电流
    abc_t i_abc = {.a = adc_values.ia, .b = adc_values.ib, .c = adc_values.ic};
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // 获取非线性磁链观测器输出的电角度和转速
    float angle_obs = fluxobserver_get_angle(&flux_observer);
    float speed_obs = fluxobserver_get_speed_rpm(&flux_observer);

    // 直接使用观测器角度参与控制
    float control_angle = angle_obs;

    // Park 变换，转换到 dq 轴
    dq_t i_dq = park_transform(i_alphabeta, control_angle);

    // 直接无感闭环：使用观测器角度和速度反馈执行速度环控制
    foc_speed_closed_loop_run(&foc_flux_handle, i_dq, control_angle, speed_obs, SPEED_LOOP_DIV);

    // 将 dq 轴电压逆变换回 αβ 轴，作为磁链观测器输入电压
    dq_t v_dq = {.d = foc_flux_handle.v_d_out, .q = foc_flux_handle.v_q_out};
    alphabeta_t v_alphabeta = ipark_transform(v_dq, control_angle);

    // 更新非线性磁链观测器输入
    flux_observer.i_alpha = i_alphabeta.alpha;
    flux_observer.i_beta = i_alphabeta.beta;
    flux_observer.u_alpha = v_alphabeta.alpha;
    flux_observer.u_beta = v_alphabeta.beta;
    fluxobserver_estimate(&flux_observer);

    // 保存编码器与观测器信息，便于上位机调试观察
    as5047_update_speed();
    speed_rpm_actual_temp = as5047_get_speed_rpm();
    angle_el_actual_temp = as5047_get_angle_rad() - foc_flux_handle.angle_offset;
    speed_rpm_observer_temp = speed_obs;
    angle_el_observer_temp = angle_obs;
}

/**
 * @brief 非线性磁链观测器零速启动初始化
 * @param target_speed_rpm 目标转速，单位 rpm
 */
void sensorless_flux_observer_init(float target_speed_rpm)
{
    // 初始化电流环和速度环 PID
    pid_init(&pid_id, 0.017f, 0.002826f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_iq, 0.017f, 0.002826f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_speed, 0.05f, 0.00002f, -2.0f, 2.0f);

    // 初始化 FOC 句柄
    foc_init(&foc_flux_handle, &pid_id, &pid_iq, &pid_speed);

    // 初始化非线性磁链观测器参数
    fluxobserver_init(&flux_observer,
                      0.12f,
                      0.00003f,
                      0.00114f,
                      7.0f,
                      0.0001f,
                      200.0e6f,
                      100.0f,
                      0.01f);

    // 设置 FOC 目标值
    foc_set_target_id(&foc_flux_handle, 0.0f);
    foc_set_target_speed(&foc_flux_handle, target_speed_rpm);

    // 电机转子对准
    foc_alignment(&foc_flux_handle);

    // 注册回调
    adc1_register_injected_callback(sensorless_flux_observer_callback);
}

/**
 * @brief 打印零速启动调试信息
 * @note 输出顺序：实际转速、观测转速、实际角度、观测角度
 */
void print_sensorless_flux_observer_info(void)
{
    // 实际角度归一化到 [0, 2π)
    float angle_actual_normalized = fmodf(angle_el_actual_temp, 2.0f * M_PI);
    if (angle_actual_normalized < 0.0f)
    {
        angle_actual_normalized += 2.0f * M_PI;
    }

    // 观测角度归一化到 [0, 2π)
    float angle_observer_normalized = fmodf(angle_el_observer_temp, 2.0f * M_PI);
    if (angle_observer_normalized < 0.0f)
    {
        angle_observer_normalized += 2.0f * M_PI;
    }

    // 转成角度值，便于 VOFA 观察
    float angle_actual_deg = angle_actual_normalized * 57.2958f;
    float angle_observer_deg = angle_observer_normalized * 57.2958f;

    // 输出调试数据到 VOFA
    float data[4] = {speed_rpm_actual_temp, speed_rpm_observer_temp, angle_actual_deg, angle_observer_deg};
    printf_vofa(data, 4);
}
