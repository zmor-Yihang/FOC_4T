#include "encoder.h"

// 编码器速度计算相关静态数据
static float current_elec_angle = 0.0f; // 当前电角度，由编码器获得（结合预测）
static float pll_phase_rad = 0.0f;      /* PLL相位估计值(单位:电角度rad) */
static float pll_speed_rad_s = 0.0f;    /* PLL速度估计值(单位:电角速度rad/s) */

/**
 * @brief 将机械角度计数值换算为电角度弧度值 0-2π
 */
static inline float encoder_covert_countToElectricalAngle(float count)
{
#if (ENCODER_COUNT_SWAP == 0) // 不颠倒
    uint16_t mech_count = count;
#else
    uint16_t mech_count = ENCODER_CPR - count;
#endif /* ENCODER_COUNT_SWAP */

    float mech_rad = mech_count * (MATH_TWO_PI / (float)ENCODER_CPR);
    return mech_rad * MOTOR_POLE_PAIRS;
}

/**
 * @brief 获取当前电角度
 * @note 编码器未通信完成时，采用预测的电角度
 */
static void encoder_get_currentElectricalAngle(void)
{
    uint16_t raw_count = 0U;

    if (as5600_poll_rawCount(&raw_count) != 0U)
    {
        current_elec_angle = encoder_covert_countToElectricalAngle(raw_count);
    }
    else
    {
        current_elec_angle = pll_phase_rad + pll_speed_rad_s * ENCODER_SPEED_SAMPLE_TIME;
    }

    current_elec_angle = wrap_0_2pi(current_elec_angle);
}

/**
 * @brief 初始化编码器
 */
void encoder_init(void)
{
    as5600_init();
}

/**
 * @brief PLL状态迭代
 */
void encoder_update(void)
{
    encoder_get_currentElectricalAngle(); // 获取当前电角度

    float phase_error = wrap_pm_pi(current_elec_angle - pll_phase_rad); // 计算当前相位误差
    float speed_integral_step = ENCODER_PLL_KI * phase_error * ENCODER_SPEED_SAMPLE_TIME;

    // 速度环积分抗饱和：到达限幅后仅允许反向积分释放，避免积分器继续累积
    if (!((pll_speed_rad_s >= ENCODER_PLL_SPEED_LIMIT_RAD_S && speed_integral_step > 0.0f) ||
          (pll_speed_rad_s <= -ENCODER_PLL_SPEED_LIMIT_RAD_S && speed_integral_step < 0.0f)))
    {
        pll_speed_rad_s += speed_integral_step;

        if (pll_speed_rad_s > ENCODER_PLL_SPEED_LIMIT_RAD_S)
        {
            pll_speed_rad_s = ENCODER_PLL_SPEED_LIMIT_RAD_S;
        }
        else if (pll_speed_rad_s < -ENCODER_PLL_SPEED_LIMIT_RAD_S)
        {
            pll_speed_rad_s = -ENCODER_PLL_SPEED_LIMIT_RAD_S;
        }
    }

    // 每个控制周期都使用比例校正项 + 当前速度估计推进PLL相位
    pll_phase_rad += (pll_speed_rad_s + ENCODER_PLL_KP * phase_error) * ENCODER_SPEED_SAMPLE_TIME;

    pll_phase_rad = wrap_0_2pi(pll_phase_rad);
}

/**
 * @brief 获取PLL估计的电角度[0, 2π)：rad
 */
float encoder_get_pllAngle(void)
{
    return pll_phase_rad;
}

/**
 * @brief 获取编码器和预测的电角度[0, 2π)：rad
 */
float encoder_get_encoderAngle(void)
{
    return current_elec_angle;
}

/**
 * @brief 获取PLL估计的转速: rpm
 */
float encoder_get_pllSpeed(void)
{
    return (pll_speed_rad_s / (MATH_TWO_PI * MOTOR_POLE_PAIRS)) * 60.0f;
}
