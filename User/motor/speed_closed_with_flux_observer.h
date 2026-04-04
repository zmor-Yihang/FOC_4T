#ifndef __SPEED_CLOSED_WITH_FLUX_OBSERVER_H__
#define __SPEED_CLOSED_WITH_FLUX_OBSERVER_H__

#include <stdio.h>
#include "foc.h"
#include "flux_observer.h"
#include "as5047.h"
#include "print.h"

/**
 * @brief 初始化有感速度闭环控制
 * @param speed_rpm 目标速度 (RPM)
 * @note 使用编码器反馈进行控制，非线性磁链观测器用于观测对比
 */
void speed_closed_with_flux_observer_init(float speed_rpm);

/**
 * @brief 打印编码器和非线性磁链观测器的速度、角度信息
 * @note 输出格式：编码器速度, 编码器角度, 磁链观测器速度, 磁链观测器角度
 */
void print_speed_flux_observer_info(void);

#endif /* __SPEED_CLOSED_WITH_FLUX_OBSERVER_H__ */
