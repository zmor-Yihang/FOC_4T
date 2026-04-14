#include "angle_utils.h"

#define ANGLE_UTILS_TWO_PI 6.28318530718f

float angle_wrap_pm_pi(float angle_rad)
{
    while (angle_rad > (ANGLE_UTILS_TWO_PI * 0.5f))
    {
        angle_rad -= ANGLE_UTILS_TWO_PI;
    }

    while (angle_rad < -(ANGLE_UTILS_TWO_PI * 0.5f))
    {
        angle_rad += ANGLE_UTILS_TWO_PI;
    }

    return angle_rad;
}

float angle_wrap_0_2pi(float angle_rad)
{
    while (angle_rad >= ANGLE_UTILS_TWO_PI)
    {
        angle_rad -= ANGLE_UTILS_TWO_PI;
    }

    while (angle_rad < 0.0f)
    {
        angle_rad += ANGLE_UTILS_TWO_PI;
    }

    return angle_rad;
}
