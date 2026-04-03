#include "flux_observer.h"

// 常量定义
#define INV_SQRT3 (0.577350269f) // 1/sqrt(3)
#define TWO_PI (6.283185307f)    // 2*PI

/**
 * @brief 初始化非线性磁链观测器
 */
void fluxobserver_init(fluxobserver_t *obs, float rs, float ls, float psi_m, float poles, float ts, float gamma, float pll_fc, float k_speed_lpf)
{
    // 电机参数
    obs->rs = rs;
    obs->ls = ls;
    obs->psi_m = psi_m;
    obs->poles = poles;
    obs->ts = ts;

    // 观测器参数
    obs->gamma = gamma;
    obs->k_speed_lpf = k_speed_lpf;

    // 初始化状态量为0
    obs->xhat_alpha = 0.0f;
    obs->xhat_beta = 0.0f;

    // 初始化角度和速度
    obs->theta_est = 0.0f;
    obs->z1 = 0.0f; // PLL角度状态
    obs->z2 = 0.0f; // PLL积分状态
    obs->speed_rad_s = 0.0f;
    obs->speed_est = 0.0f;
    obs->speed_est_filt = 0.0f;

    // 初始化PLL PI控制器 - 使用 pid_init 初始化
    // 典型值：Kp = 2 * ζ * ωn, Ki = ωn^2
    // ωn = 2π * fc
    float wn = 2.0f * M_PI * pll_fc;
    float zeta = 0.5f; // 阻尼系数
    float kp = 2.0f * zeta * wn;
    float ki = wn * wn * ts; // 注意：pid_calculate 内部不乘 ts，所以这里预乘

    // 速度范围：假设最大 ±10000 RPM
    // 转换为电角速度：ω_elec = RPM * 2π * poles / 60
    float max_rpm = 10000.0f;
    float max_speed_rad_s = max_rpm * 2.0f * M_PI * poles / 60.0f;

    pid_init(&obs->pll, kp, ki, -max_speed_rad_s, max_speed_rad_s);

    // 初始化输入为0
    obs->i_alpha = 0.0f;
    obs->i_beta = 0.0f;
    obs->u_alpha = 0.0f;
    obs->u_beta = 0.0f;
}

/**
 * @brief 运行非线性磁链观测器
 */
void fluxobserver_estimate(fluxobserver_t *obs)
{
    // y = -Rs*i + u 
    float y_alpha = -obs->rs * obs->i_alpha + obs->u_alpha;
    float y_beta = -obs->rs * obs->i_beta + obs->u_beta;

    // η = xhat - L*i 
    float eta_alpha = obs->xhat_alpha - obs->ls * obs->i_alpha;
    float eta_beta = obs->xhat_beta - obs->ls * obs->i_beta;

    // r2 = ||η||^2 = η_alpha^2 + η_beta^2
    float r2 = eta_alpha * eta_alpha + eta_beta * eta_beta;

    // s = psi_m^2 - r2
    // s > 0: 估计磁链在圆内
    // s < 0: 估计磁链在圆外
    float psi_m2 = obs->psi_m * obs->psi_m;
    float s = psi_m2 - r2;

    // dxhat = y + 0.5 * gamma * η * s
    float dxhat_alpha = y_alpha + 0.5f * obs->gamma * eta_alpha * s;
    float dxhat_beta = y_beta + 0.5f * obs->gamma * eta_beta * s;

    // xhat[k+1] = xhat[k] + Ts * dxhat
    obs->xhat_alpha += obs->ts * dxhat_alpha;
    obs->xhat_beta += obs->ts * dxhat_beta;

    // 使用 η 作为磁链估算值来计算角度
    // theta_hat = atan2(η_beta, η_alpha)
    obs->theta_est = atan2f(eta_beta, eta_alpha);

    // 角度误差 e_theta = theta_hat - z1
    float e_theta = obs->theta_est - obs->z1;
    while (e_theta > M_PI)
        e_theta -= 2.0f * M_PI;
    while (e_theta < -M_PI)
        e_theta += 2.0f * M_PI;

    // 使用 PI 控制器调节速度
    float speed_rad_s = pid_calculate(&obs->pll, e_theta, 0.0f);

    // 更新PLL角度状态 z1
    obs->z1 += obs->ts * speed_rad_s;
    while (obs->z1 > M_PI)
        obs->z1 -= 2.0f * M_PI;
    while (obs->z1 < -M_PI)
        obs->z1 += 2.0f * M_PI;

    obs->speed_rad_s = speed_rad_s;

    // 电角速度 -> 机械转速 (rpm)
    // omega_mech = omega_elec / poles
    // rpm = omega_mech * 60 / (2*pi) = omega_elec * 60 / (2*pi*poles)
    float speed_rpm = obs->speed_rad_s * 60.0f / (TWO_PI * obs->poles);
    obs->speed_est = speed_rpm;

    // 低通滤波
    obs->speed_est_filt = obs->speed_est_filt * (1.0f - obs->k_speed_lpf) + speed_rpm * obs->k_speed_lpf;
}

/**
 * @brief 获取估算的角度 (rad)
 * @note 返回PLL输出的平滑角度z1，而非直接从磁链计算的theta_est
 *       这样可以保证角度连续平滑，避免静差问题
 */
float fluxobserver_get_angle(fluxobserver_t *obs)
{
    return obs->z1;
}

/**
 * @brief 获取估算的速度 (rpm)
 */
float fluxobserver_get_speed_rpm(fluxobserver_t *obs)
{
    return obs->speed_est_filt;
}
