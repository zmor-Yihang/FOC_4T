#include "speed_closed_with_flux_observer.h"

// FOC 控制句柄
static foc_t foc_handle;

// 非线性磁链观测器实例
static fluxobserver_t flux_observer;

// PID 实例
static pid_controller_t pid_id;
static pid_controller_t pid_iq;
static pid_controller_t pid_speed;

// 打印用变量
static float speed_rpm_encoder = 0.0f;
static float angle_el_encoder = 0.0f;
static float speed_rpm_flux = 0.0f;
static float angle_el_flux = 0.0f;
static float eta_alpha_flux = 0.0f;
static float eta_beta_flux = 0.0f;

static void speed_closed_with_flux_observer_callback(void)
{
    // 更新编码器速度
    as5047_update_speed();

    // 获取编码器角度和速度
    float angle_el = as5047_get_angle_rad() - foc_handle.angle_offset;
    float speed_feedback = as5047_get_speed_rpm();

    // 获取电流反馈值
    adc_values_t adc_values;
    adc1_get_injected_values(&adc_values);

    // Clark 变换
    abc_t i_abc = {.a = adc_values.ia, .b = adc_values.ib, .c = adc_values.ic};
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // Park 变换（使用编码器角度）
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 速度闭环控制（使用编码器反馈）
    foc_speed_closed_loop_run(&foc_handle, i_dq, angle_el, speed_feedback, AS5047_SPEED_CALC_DIV);

    // 获取输出电压
    dq_t v_dq = {.d = foc_handle.v_d_out, .q = foc_handle.v_q_out};

    // 反Park变换（使用编码器角度）
    alphabeta_t v_alphabeta = ipark_transform(v_dq, angle_el);

    // 更新非线性磁链观测器
    flux_observer.i_alpha = i_alphabeta.alpha;
    flux_observer.i_beta = i_alphabeta.beta;
    flux_observer.u_alpha = v_alphabeta.alpha;
    flux_observer.u_beta = v_alphabeta.beta;
    fluxobserver_estimate(&flux_observer);

    // 获取磁链观测器观测值
    float angle_flux = fluxobserver_get_angle(&flux_observer);
    float speed_flux = fluxobserver_get_speed_rpm(&flux_observer);

    // 保存打印数据
    speed_rpm_encoder = speed_feedback;
    angle_el_encoder = angle_el;
    speed_rpm_flux = speed_flux;
    angle_el_flux = angle_flux;

    // 计算 eta = xhat - L*i 用于调试显示
    eta_alpha_flux = flux_observer.xhat_alpha - flux_observer.ls * i_alphabeta.alpha;
    eta_beta_flux = flux_observer.xhat_beta - flux_observer.ls * i_alphabeta.beta;
}

void speed_closed_with_flux_observer_init(float speed_rpm)
{
    // PID 初始化
    pid_init(&pid_id, 0.017f, 0.002826f, -U_DC / 3.0f, U_DC / 3.0f);
    pid_init(&pid_iq, 0.017f, 0.002826f, -U_DC / 3.0f, U_DC / 3.0f);
    pid_init(&pid_speed, 0.05f, 0.00002f, -4.0f, 4.0f);

    // FOC 初始化
    foc_init(&foc_handle, &pid_id, &pid_iq, &pid_speed);

    // 初始化非线性磁链观测器
    // 参数：Rs, Ls, Psi_m, 极对数, 采样周期, gamma增益, PLL截止频率, 速度滤波系数
    fluxobserver_init(&flux_observer,
                      0.12f,    // rs: 定子电阻 (Ω)
                      0.00003f, // ls: 定子电感 (H)
                      0.00114f, // psi_m: 永磁体磁链 (Wb)
                      7.0f,     // poles: 极对数
                      0.0001f,  // ts: 采样周期 (s) = 10kHz
                      200.0e6f, // gamma: 非线性增益
                      100.0f,    // pll_fc: PLL截止频率 (Hz)
                      0.05f);   // k_speed_lpf: 速度滤波系数

    // 设置目标值
    foc_set_target_id(&foc_handle, 0.0f);
    foc_set_target_speed(&foc_handle, speed_rpm);

    // 零点对齐
    foc_alignment(&foc_handle);

    // 注册回调函数
    adc1_register_injected_callback(speed_closed_with_flux_observer_callback);
}

void print_speed_flux_observer_info(void)
{
    // 归一化角度到 [0, 2π) 范围
    float angle_encoder_normalized = fmodf(angle_el_encoder, 2.0f * M_PI);
    if (angle_encoder_normalized < 0.0f)
    {
        angle_encoder_normalized += 2.0f * M_PI;
    }

    float angle_flux_normalized = fmodf(angle_el_flux, 2.0f * M_PI);
    if (angle_flux_normalized < 0.0f)
    {
        angle_flux_normalized += 2.0f * M_PI;
    }

    // 转换为角度 (0-360°)
    float angle_encoder_deg = angle_encoder_normalized * 57.2958f;
    float angle_flux_deg = angle_flux_normalized * 57.2958f;

    // 发送 JustFloat 协议数据
    float data[6] = {speed_rpm_encoder, speed_rpm_flux, angle_encoder_deg, angle_flux_deg, eta_alpha_flux, eta_beta_flux};
    printf_vofa(data, 6);
}
