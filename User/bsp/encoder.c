#include "encoder.h"
#include "angle_utils.h"

// 编码器速度计算相关静态数据
static float current_elec_angle = 0.0f; // 当前电角度，由编码器获得（结合预测）
static float pll_phase_rad = 0.0f;      /* PLL相位估计值(单位:电角度rad) */
static float pll_speed_rad_s = 0.0f;    /* PLL速度估计值(单位:电角速度rad/s) */

// 编码器异步读取接收缓冲区
static uint8_t i2c_rx_buf[2] = {0};

/**
 * @brief 拼接I2C接收缓冲区得到原始计数 0-4965
 */
static inline uint16_t encoder_build_raw_count(const uint8_t *recv_buffer)
{
    return ((((uint16_t)recv_buffer[0]) << 8) | recv_buffer[1]) & 0x0FFF;
}

/**
 * @brief 将机械角度计数值换算为电角度弧度值 0-2π
 */
static inline float encoder_count_to_elec_rad(float count)
{
    uint16_t mech_count = ENCODER_CPR - count;
    float mech_rad = mech_count * (ENCODER_TWO_PI / (float)ENCODER_CPR);
    return mech_rad * MOTOR_POLE_PAIRS;
}

/**
 * @brief 获取当前电角度
 * @note 编码器未通信完成时，采用预测的电角度
 */
static void encoder_get_current_elec_angle(void)
{
    switch (i2c_read_get_state())
    {
    case I2C_READ_STATE_BUSY:
        // 预测角度
        current_elec_angle = pll_phase_rad + pll_speed_rad_s * ENCODER_SPEED_SAMPLE_TIME;
        break;
    case I2C_READ_STATE_DONE:
        // 读取编码器角度
        current_elec_angle = encoder_count_to_elec_rad(encoder_build_raw_count(i2c_rx_buf));
        i2c_read_bytes_async(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, i2c_rx_buf, 2);
        break;
    }
    current_elec_angle = angle_wrap_0_2pi(current_elec_angle);
}

/**
 * * @brief 初始化编码器
 */
void encoder_init(void)
{
    i2c_init();
    i2c_read_bytes_async(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, i2c_rx_buf, 2);
    HAL_Delay(1); // 确保I2C读写完成
}

/**
 * @brief PLL状态迭代
 */
void encoder_update(void)
{
    encoder_get_current_elec_angle(); // 获取当前电角度

    float phase_error = angle_wrap_pm_pi(current_elec_angle - pll_phase_rad); // 计算当前相位误差

    // 速度环积分项：根据电角度误差更新电角速度估计(rad/s)
    pll_speed_rad_s += ENCODER_PLL_KI * phase_error * ENCODER_SPEED_SAMPLE_TIME;

    // 每个控制周期都使用比例校正项 + 当前速度估计推进PLL相位
    pll_phase_rad += (pll_speed_rad_s + ENCODER_PLL_KP * phase_error) * ENCODER_SPEED_SAMPLE_TIME;

    pll_phase_rad = angle_wrap_0_2pi(pll_phase_rad);
}

/**
 * @brief 获取PLL估计的电角度[0, 2π)：rad
 */
float encoder_get_angle(void)
{
    return pll_phase_rad;
}

/**
 * @brief 获取编码器和预测的电角度[0, 2π)：rad
 */
float encoder_get_encoder_angle(void)
{
    return current_elec_angle;
}

/**
 * @brief 获取PLL估计的转速: rpm
 */
float encoder_get_speed(void)
{
    return (pll_speed_rad_s / (ENCODER_TWO_PI * MOTOR_POLE_PAIRS)) * 60.0f;
}