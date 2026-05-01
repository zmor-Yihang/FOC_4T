#include "utils.h"

uint8_t divider_ready(uint8_t *counter, uint8_t divider)
{
    (*counter)++; // 计数器加一

    // 当计数器达到分频系数时，表示对应任务周期到达，重置计数器并返回 1；否则返回 0 表示继续等待
    if (*counter >= divider)
    {
        *counter = 0U;
        return 1U;
    }

    return 0U;
}

float utils_clampf(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

float wrap_pm_pi(float angle_rad)
{
    while (angle_rad > (MATH_TWO_PI * 0.5f))
    {
        angle_rad -= MATH_TWO_PI;
    }

    while (angle_rad < -(MATH_TWO_PI * 0.5f))
    {
        angle_rad += MATH_TWO_PI;
    }

    return angle_rad;
}

float wrap_0_2pi(float angle_rad)
{
    while (angle_rad >= MATH_TWO_PI)
    {
        angle_rad -= MATH_TWO_PI;
    }

    while (angle_rad < 0.0f)
    {
        angle_rad += MATH_TWO_PI;
    }

    return angle_rad;
}

float angle_compensate_delay(float angle_rad, float speed_rad_s, float delay_s)
{
    float compensated_angle = angle_rad + speed_rad_s * delay_s;
    return wrap_0_2pi(compensated_angle);
}
