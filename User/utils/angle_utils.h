#ifndef __ANGLE_UTILS_H__
#define __ANGLE_UTILS_H__

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

#endif /* __ANGLE_UTILS_H__ */
