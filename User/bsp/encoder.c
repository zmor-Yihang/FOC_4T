#include "encoder.h"

// 编码器速度计算相关静态数据
static struct
{
    uint16_t raw_count_cached; /* 最近一次采样得到的原始计数缓存 */
    float phase_est_count;     /* PLL相位估计值(单位:count) */
    float speed_est_count_s;   /* PLL速度估计值(单位:count/s) */
    float speed_rpm;           /* 当前原始转速(RPM) */
} encoder_speed_data = {0, 0.0f, 0.0f, 0.0f};

// 编码器异步读取接收缓冲区
static uint8_t encoder_i2c_recv_buffer[2] = {0};

static uint16_t encoder_build_raw_count(const uint8_t *recv_buffer)
{
    return ((((uint16_t)recv_buffer[0]) << 8) | recv_buffer[1]) & 0x0FFF;
}

static uint16_t encoder_predict_raw_count(void)
{
    float predicted_count = (float)encoder_speed_data.raw_count_cached +
                            encoder_speed_data.speed_est_count_s * ENCODER_SPEED_SAMPLE_TIME;

    while (predicted_count >= ENCODER_CPR)
    {
        predicted_count -= ENCODER_CPR;
    }
    while (predicted_count < 0.0f)
    {
        predicted_count += ENCODER_CPR;
    }

    return (uint16_t)predicted_count;
}

/**
 * @brief 读取AS5600原始角度计数值
 * @return 原始角度计数，范围0~4095
 * @note  异步拉取I2C新样本；若当前周期还未完成，则退化为速度预测值
 */
static uint16_t encoder_get_raw_count(void)
{
    uint16_t raw_count;

    switch (i2c_read_get_state())
    {
    case I2C_READ_STATE_DONE: // 有新样本时优先消费
        raw_count = encoder_build_raw_count(encoder_i2c_recv_buffer);
        i2c_read_set_idle();
        break;

    case I2C_READ_STATE_IDLE:
    case I2C_READ_STATE_BUSY: // 当前周期若还未收到新样本，则退化为速度预测值
        raw_count = encoder_predict_raw_count();
        break;
    }

    if (i2c_read_get_state() == I2C_READ_STATE_IDLE)
    {
        i2c_read_bytes_it(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, encoder_i2c_recv_buffer, 2);
    }

    return raw_count;
}

void encoder_init(void)
{
    // 初始化AS5600所使用的I2C外设
    i2c_init();

    // init阶段先阻塞读取一次编码器并完成PLL状态对齐
    i2c_read_bytes(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, encoder_i2c_recv_buffer, 2);
    uint16_t current_count = encoder_build_raw_count(encoder_i2c_recv_buffer);

    // 初始化速度估算状态变量
    encoder_speed_data.raw_count_cached = current_count;
    encoder_speed_data.phase_est_count = (float)current_count;
    encoder_speed_data.speed_est_count_s = 0.0f;
    encoder_speed_data.speed_rpm = 0.0f;

    // 提前发起第一次异步读取，供后续控制周期直接消费
    i2c_read_bytes_it(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, encoder_i2c_recv_buffer, 2);
}

/**
 * @brief 更新编码器转速估算值
 * @note  需要在固定周期定时中断中调用
 */
void encoder_update(void)
{
    // 读取当前原始角度计数值
    uint16_t current_count = encoder_get_raw_count();
    encoder_speed_data.raw_count_cached = current_count;

    // PLL: 根据测量相位与估计相位的误差，联合修正相位和速度
    float phase_error = (float)current_count - encoder_speed_data.phase_est_count;
    float half_cpr = ENCODER_CPR * 0.5f;

    // 处理过零点误差，保证相位误差为最短路径
    if (phase_error > half_cpr)
    {
        phase_error -= ENCODER_CPR;
    }
    else if (phase_error < -half_cpr)
    {
        phase_error += ENCODER_CPR;
    }

    // 速度环积分项：根据相位误差更新速度估计(count/s)
    encoder_speed_data.speed_est_count_s += ENCODER_PLL_KI * phase_error * ENCODER_SPEED_SAMPLE_TIME;

    // 限幅避免异常采样导致速度估计失控
    if (encoder_speed_data.speed_est_count_s > ENCODER_PLL_MAX_SPEED_COUNT_S)
    {
        encoder_speed_data.speed_est_count_s = ENCODER_PLL_MAX_SPEED_COUNT_S;
    }
    else if (encoder_speed_data.speed_est_count_s < -ENCODER_PLL_MAX_SPEED_COUNT_S)
    {
        encoder_speed_data.speed_est_count_s = -ENCODER_PLL_MAX_SPEED_COUNT_S;
    }

    // 相位环比例项+速度前馈项，更新相位估计(count)
    encoder_speed_data.phase_est_count += (encoder_speed_data.speed_est_count_s + ENCODER_PLL_KP * phase_error) * ENCODER_SPEED_SAMPLE_TIME;

    // 将估计相位限制到[0, CPR)范围
    while (encoder_speed_data.phase_est_count >= ENCODER_CPR)
    {
        encoder_speed_data.phase_est_count -= ENCODER_CPR;
    }
    while (encoder_speed_data.phase_est_count < 0.0f)
    {
        encoder_speed_data.phase_est_count += ENCODER_CPR;
    }
}

/**
 * @brief 获取当前电角度弧度值
 * @return 当前电角度，范围[0, 2π)
 */
float encoder_get_angle_rad(void)
{
    float elec_angle;

    // 使用最近一次采样缓存值换算电角度，避免重复I2C读取
    elec_angle = ((float)encoder_speed_data.raw_count_cached / ENCODER_CPR) * MOTOR_POLE_PAIRS * ENCODER_TWO_PI * ENCODER_DIRECTION;

    // 将电角度限制到[0, 2π)范围内
    while (elec_angle >= ENCODER_TWO_PI)
    {
        elec_angle -= ENCODER_TWO_PI;
    }

    while (elec_angle < 0.0f)
    {
        elec_angle += ENCODER_TWO_PI;
    }

    return elec_angle;
}

/**
 * @brief 获取转速
 * @return 最近一次更新得到的滤波转速，单位RPM
 */
float encoder_get_speed_rpm(void)
{
    return (encoder_speed_data.speed_est_count_s / ENCODER_CPR) * 60.0f * ENCODER_DIRECTION;
}
