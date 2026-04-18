#include "angle_utils.h"
#include "../app/user_config.h"

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
