#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>
#include "../app/user_config.h"

/**
 * @brief 分频调度判断函数
 * @param counter  分频计数器指针，由调用方保存计数状态
 * @param divider  分频系数，表示每调用 divider 次返回一次 1
 * @return uint8_t 返回 1 表示本次应执行对应任务，返回 0 表示本次跳过
 */
uint8_t divider_ready(uint8_t *counter, uint8_t divider);

/**
 * @brief 将角度包络到[-π, π]范围
 * @param angle_rad 待处理角度，单位rad
 * @return 包络后的最短路径角度误差
 * @note  主要用于误差计算，避免跨零点时角度突变
 */
float wrap_pm_pi(float angle_rad);

/**
 * @brief 将角度包络到[0, 2π)范围
 * @param angle_rad 待处理角度，单位rad
 * @return 归一化后的角度
 * @note  用于保证角度状态始终落在单圈范围内
 */
float wrap_0_2pi(float angle_rad);

/**
 * @brief 电角度拍延时补偿
 * @param angle_rad 当前电角度，单位rad
 * @param speed_rad_s 当前电角速度，单位rad/s
 * @param delay_s 总等效延时，单位s
 * @return 补偿后的电角度[0, 2π)
 */
float angle_compensate_delay(float angle_rad, float speed_rad_s, float delay_s);

#endif /* __UTILS_H__ */
