#include "encoder.h"

// 编码器速度计算相关静态数据
static float current_elec_angle = 0.0f; // 当前电角度，由编码器获得（结合预测）
static float current_mech_angle = 0.0f; // 当前机械单圈角度[0, 2π)
static float mech_position = 0.0f;      // 当前机械多圈位置(rad)
static float last_mech_angle = 0.0f;    // 上一次机械单圈角度(rad)
static uint8_t mech_position_inited = 0U;
static float pll_phase_rad = 0.0f;   /* PLL相位估计值(单位:电角度rad) */
static float pll_speed_rad_s = 0.0f; /* PLL速度估计值(单位:电角速度rad/s) */

/**
 * @brief 将计数值换算为机械角度（rad） 0-2π
 */
static inline float encoder_convert_countToMechanicalAngle(float count)
{
#if (ENCODER_COUNT_SWAP == 0) // 不颠倒
    uint16_t mech_count = count;
#else
    uint16_t mech_count = ENCODER_CPR - count;
#endif /* ENCODER_COUNT_SWAP */

    return mech_count * (MATH_TWO_PI / (float)ENCODER_CPR);
}

/**
 * @brief 更新机械多圈位置
 */
static void encoder_update_mechanicalPosition(float mech_angle)
{
    if (mech_position_inited == 0U)
    {
        mech_position = mech_angle;
        last_mech_angle = mech_angle;
        mech_position_inited = 1U;
        return;
    }

    float delta_angle = wrap_pm_pi(mech_angle - last_mech_angle);
    mech_position += delta_angle;
    last_mech_angle = mech_angle;
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
        current_mech_angle = encoder_convert_countToMechanicalAngle(raw_count);
        current_elec_angle = current_mech_angle * MOTOR_POLE_PAIRS;
    }
    else
    {
        current_mech_angle = wrap_0_2pi((pll_phase_rad + pll_speed_rad_s * ENCODER_SPEED_SAMPLE_TIME) / MOTOR_POLE_PAIRS);
        current_elec_angle = pll_phase_rad + pll_speed_rad_s * ENCODER_SPEED_SAMPLE_TIME;
    }

    encoder_update_mechanicalPosition(current_mech_angle);
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
        pll_speed_rad_s = utils_clampf(pll_speed_rad_s, -ENCODER_PLL_SPEED_LIMIT_RAD_S, ENCODER_PLL_SPEED_LIMIT_RAD_S);
    }

    // 每个控制周期都使用比例校正项 + 当前速度估计推进PLL相位
    pll_phase_rad += (pll_speed_rad_s + ENCODER_PLL_KP * phase_error) * ENCODER_SPEED_SAMPLE_TIME;

    pll_phase_rad = wrap_0_2pi(pll_phase_rad);
}

/**
 * @brief 获取编码器和预测的电角度[0, 2π)：rad
 */
float encoder_get_encoderAngle(void)
{
#if (ENCODER_PLL_ANGLE_COMP_ENABLE == 1)
    return angleUtils_compensate_delay(current_elec_angle, pll_speed_rad_s, ENCODER_PLL_ANGLE_COMP_DELAY_S);
#else
    return current_elec_angle;
#endif /* ENCODER_PLL_ANGLE_COMP_ENABLE */
}

/**
 * @brief 获取PLL估计的电角度[0, 2π)：rad
 */
float encoder_get_pllAngle(void)
{
#if (ENCODER_PLL_ANGLE_COMP_ENABLE == 1)
    return angleUtils_compensate_delay(pll_phase_rad, pll_speed_rad_s, ENCODER_PLL_ANGLE_COMP_DELAY_S);
#else
    return pll_phase_rad;
#endif /* ENCODER_PLL_ANGLE_COMP_ENABLE */
}

/**
 * @brief 获取PLL估计的转速: rpm
 */
float encoder_get_pllSpeed(void)
{
    return (pll_speed_rad_s / (MATH_TWO_PI * MOTOR_POLE_PAIRS)) * 60.0f;
}

/**
 * @brief 获取机械单圈角度[0, 2π)：rad
 */
float encoder_get_mechanicalAngle(void)
{
    return current_mech_angle;
}

/**
 * @brief 获取机械多圈位置：rad
 */
float encoder_get_mechanicalPosition(void)
{
    return mech_position;
}

/**
 * @brief 获取机械多圈位置：rev
 */
float encoder_get_mechanicalPositionRev(void)
{
    return mech_position / MATH_TWO_PI;
}

/**
 * @brief 重置机械多圈位置零点
 * @param position_rad 重置后的当前位置(rad)
 */
void encoder_reset_mechanicalPosition(float position_rad)
{
    mech_position = position_rad;
    last_mech_angle = current_mech_angle;
    mech_position_inited = 1U;
}
