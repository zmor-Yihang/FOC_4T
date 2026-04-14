#include "encoder.h"
#include "angle_utils.h"

// 编码器速度计算相关静态数据
static uint16_t raw_cnt = 0;         /* 最近一次采样得到的原始计数缓存 */
static float pll_phase_rad = 0.0f;   /* PLL相位估计值(单位:电角度rad) */
static float pll_speed_rad_s = 0.0f; /* PLL速度估计值(单位:电角速度rad/s) */

// 编码器异步读取接收缓冲区
static uint8_t i2c_rx_buf[2] = {0};

// 计数值获取
static inline uint16_t encoder_build_raw_count(const uint8_t *recv_buffer)
{
    return ((((uint16_t)recv_buffer[0]) << 8) | recv_buffer[1]) & 0x0FFF;
}

/**
 * @brief 将机械角度计数值换算为电角度弧度值
 */
static inline float encoder_count_to_elec_rad(float count)
{
    return count * ((ENCODER_TWO_PI * MOTOR_POLE_PAIRS) / ENCODER_CPR) * ENCODER_DIRECTION;
}

/**
 * @brief 如果I2C空闲则发起下一次AS5600异步读角度
 */
static void encoder_start_async_read_if_idle(void)
{
    if (i2c_read_get_state() == I2C_READ_STATE_IDLE)
    {
        i2c_read_bytes_it(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, i2c_rx_buf, 2);
    }
}

/**
 * @brief 消费一次已经完成的AS5600新样本
 * @return 1表示本周期拿到了新测量值，0表示没有新样本
 */
static uint8_t encoder_consume_raw_sample(void)
{
    if (i2c_read_get_state() == I2C_READ_STATE_DONE)
    {
        raw_cnt = encoder_build_raw_count(i2c_rx_buf);
        i2c_read_set_idle();
        return 1U;
    }

    return 0U;
}

void encoder_init(void)
{
    // 初始化AS5600所使用的I2C外设
    i2c_init();

    // init阶段先阻塞读取一次编码器并完成PLL状态对齐
    i2c_read_bytes(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, i2c_rx_buf, 2);
    raw_cnt = encoder_build_raw_count(i2c_rx_buf);

    // 初始化速度估算状态变量
    pll_phase_rad = angle_wrap_0_2pi(encoder_count_to_elec_rad((float)raw_cnt));
    pll_speed_rad_s = 0.0f;

    // 提前发起第一次异步读取，供后续控制周期直接消费
    encoder_start_async_read_if_idle();
}

/**
 * @brief 更新编码器PLL状态
 * @note  控制主角度使用PLL相位；有新样本时做校正，无新样本时按估计速度连续推进
 */
void encoder_update(void)
{
    float phase_error = 0.0f;
    uint8_t has_new_sample = encoder_consume_raw_sample();

    if (has_new_sample != 0U)
    {
        float current_elec_angle = angle_wrap_0_2pi(encoder_count_to_elec_rad((float)raw_cnt));
        phase_error = angle_wrap_pm_pi(current_elec_angle - pll_phase_rad);

        // 速度环积分项：根据电角度误差更新电角速度估计(rad/s)
        pll_speed_rad_s += ENCODER_PLL_KI * phase_error * ENCODER_SPEED_SAMPLE_TIME;
    }

    // 限幅避免异常采样导致速度估计失控
    if (pll_speed_rad_s > ENCODER_PLL_MAX_SPEED)
    {
        pll_speed_rad_s = ENCODER_PLL_MAX_SPEED;
    }
    else if (pll_speed_rad_s < -ENCODER_PLL_MAX_SPEED)
    {
        pll_speed_rad_s = -ENCODER_PLL_MAX_SPEED;
    }

    if (has_new_sample != 0U)
    {
        // 有新样本时，用比例校正项 + 当前速度估计推进控制相位
        pll_phase_rad += (pll_speed_rad_s + ENCODER_PLL_KP * phase_error) * ENCODER_SPEED_SAMPLE_TIME;
    }
    else
    {
        // 没有新样本时，直接按当前估计速度推进控制相位
        pll_phase_rad += pll_speed_rad_s * ENCODER_SPEED_SAMPLE_TIME;
    }

    pll_phase_rad = angle_wrap_0_2pi(pll_phase_rad);

    // 保持后台异步拉取最新传感器样本，供后续周期校正PLL
    encoder_start_async_read_if_idle();
}

/**
 * @brief 获取当前电角度弧度值
 * @return 当前电角度，范围[0, 2π)
 */
float encoder_get_angle_rad(void)
{
    // 使用最近一次采样缓存值换算电角度，避免重复I2C读取
    return angle_wrap_0_2pi(encoder_count_to_elec_rad((float)raw_cnt));
}

/**
 * @brief 阻塞读取一次AS5600原始角度并返回电角度
 * @return 当前电角度，范围[0, 2π)
 * @note  主要用于上电对齐阶段，确保每次取到实时角度
 */
float encoder_get_angle_rad_blocking(void)
{
    uint8_t raw_buf[2];

    i2c_read_bytes(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, raw_buf, 2);
    raw_cnt = encoder_build_raw_count(raw_buf);

    return angle_wrap_0_2pi(encoder_count_to_elec_rad((float)raw_cnt));
}

/**
 * @brief 将PLL状态同步到当前编码器角度
 * @note  主要用于上电对齐结束后，避免速度估计从旧相位突跳
 */
void encoder_sync_pll_to_current_angle(void)
{
    pll_phase_rad = angle_wrap_0_2pi(encoder_count_to_elec_rad((float)raw_cnt));
    pll_speed_rad_s = 0.0f;

    // 对齐阶段可能残留一帧旧的异步读完成标志，这里清掉并重新拉起新样本
    i2c_read_set_idle();
    encoder_start_async_read_if_idle();
}

/**
 * @brief 获取PLL估计的电角度弧度值
 * @return PLL估计电角度，范围[0, 2π)
 */
float encoder_get_pll_angle_rad(void)
{
    return angle_wrap_0_2pi(pll_phase_rad);
}

/**
 * @brief 获取转速
 * @return 最近一次更新得到的滤波转速，单位RPM
 */
float encoder_get_speed_rpm(void)
{
    return (pll_speed_rad_s / (ENCODER_TWO_PI * MOTOR_POLE_PAIRS)) * 60.0f;
}
