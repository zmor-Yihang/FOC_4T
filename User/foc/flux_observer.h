#ifndef __FLUX_OBSERVER_H_
#define __FLUX_OBSERVER_H_

#include <math.h>
#include "pid.h"

/**
 * @brief 非线性磁链观测器结构体
 * @note 基于磁链圆约束的非线性观测器
 */
typedef struct
{
    float i_alpha; // 实测电流 alpha
    float i_beta;  // 实测电流 beta
    float u_alpha; // 电压 alpha
    float u_beta;  // 电压 beta

    float rs;    // 定子电阻 (Ω)
    float ls;    // 定子电感 (H)
    float psi_m; // 永磁体磁链 (Wb)
    float ts;    // 控制周期 (s)
    float poles; // 电机极对数

    float gamma;    // 非线性增益
    float k_pll_kp; // PLL KP
    float k_pll_ki; // PLL KI

    float xhat_alpha; // 估算扩展磁链 alpha
    float xhat_beta;  // 估算扩展磁链 beta

    float theta_est;      // 估算角度 (rad)
    float z1;             // PLL角度积分状态
    float z2;             // PLL误差积分状态
    float speed_rad_s;    // 估算电角速度 (rad/s)
    float speed_est;      // 估算速度 (rpm)
    float speed_est_filt; // 滤波后的速度 (rpm)
    float k_speed_lpf;    // 速度低通滤波系数

    // PLL PI 控制器
    pid_controller_t pll;

} fluxobserver_t;

/**
 * @brief 初始化非线性磁链观测器
 * @param obs 观测器句柄
 * @param rs 定子电阻 (Ω)
 * @param ls 定子电感 (H)
 * @param psi_m 永磁体磁链 (Wb)
 * @param poles 极对数
 * @param ts 采样周期 (s)
 * @param gamma 非线性增益
 * @param pll_fc PLL截止频率 (Hz)
 * @param k_speed_lpf 速度低通滤波系数
 */
void fluxobserver_init(fluxobserver_t *obs, float rs, float ls, float psi_m, float poles, float ts, float gamma, float pll_fc, float k_speed_lpf);

/**
 * @brief 运行非线性磁链观测器
 * @param obs 观测器句柄
 */
void fluxobserver_estimate(fluxobserver_t *obs);

float fluxobserver_get_angle(fluxobserver_t *obs);

float fluxobserver_get_speed_rpm(fluxobserver_t *obs);

#endif
