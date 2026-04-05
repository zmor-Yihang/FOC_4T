#include "encoder.h"

// 编码器速度计算相关静态数据
static struct
{
    uint16_t last_count;    /* 上一次角度计数值 */
    int32_t accum_count;    /* 分频周期内累计计数增量 */
    uint16_t sample_div_cnt;/* 速度计算分频计数 */
    float speed_rpm;        /* 当前原始转速(RPM) */
    float speed_rpm_lpf;    /* 低通滤波后的转速(RPM) */
    uint8_t is_initialized; /* 速度模块初始化标志 */
} encoder_speed_data = {0, 0, 0, 0.0f, 0.0f, 0};

void encoder_init(void)
{
    // 初始化AS5600所使用的I2C外设
    i2c_init();

    // 清零速度估算状态变量
    encoder_speed_data.last_count = 0;
    encoder_speed_data.accum_count = 0;
    encoder_speed_data.sample_div_cnt = 0;
    encoder_speed_data.speed_rpm = 0.0f;
    encoder_speed_data.speed_rpm_lpf = 0.0f;
    encoder_speed_data.is_initialized = 0;
}

/**
 * @brief 读取AS5600原始角度计数值
 * @return 原始角度计数，范围0~4095
 */
static uint16_t encoder_get_raw_count(void)
{
    // 接收AS5600原始角度寄存器的高低字节
    uint8_t recv_buffer[2] = {0};

    // 从RAW ANGLE寄存器开始连续读取2字节原始角度值
    i2c_read_bytes(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, recv_buffer, 2);

    // 拼接高低字节，并取低12位得到原始角度计数值
    return ((((uint16_t)recv_buffer[0]) << 8) | recv_buffer[1]) & 0x0FFF;
}

/**
 * @brief 更新编码器转速估算值
 * @note  需要在固定周期定时中断中调用
 */
void encoder_update_speed(void)
{
    // 读取当前原始角度计数值
    uint16_t current_count = encoder_get_raw_count();

    // 首次调用时仅记录初值，不计算速度
    if (!encoder_speed_data.is_initialized)
    {
        encoder_speed_data.last_count = current_count;
        encoder_speed_data.accum_count = 0;
        encoder_speed_data.sample_div_cnt = 0;
        encoder_speed_data.speed_rpm = 0.0f;
        encoder_speed_data.speed_rpm_lpf = 0.0f;
        encoder_speed_data.is_initialized = 1;
        return;
    }

    // 计算当前采样周期内的计数增量
    int32_t delta_count = (int32_t)current_count - (int32_t)encoder_speed_data.last_count;

    // 处理过零点跳变，保证取最短路径增量
    if (delta_count > (int32_t)(ENCODER_CPR / 2.0f))
    {
        delta_count -= (int32_t)ENCODER_CPR;
    }
    else if (delta_count < -(int32_t)(ENCODER_CPR / 2.0f))
    {
        delta_count += (int32_t)ENCODER_CPR;
    }

    // 分频累加计数增量，达到设定窗口后再更新一次速度
    encoder_speed_data.accum_count += delta_count;
    encoder_speed_data.sample_div_cnt++;

    if (encoder_speed_data.sample_div_cnt >= SPEED_CAL_DIV)
    {
        float sample_time = ENCODER_SPEED_SAMPLE_TIME * (float)encoder_speed_data.sample_div_cnt;

        encoder_speed_data.speed_rpm = ((float)encoder_speed_data.accum_count / ENCODER_CPR) * 60.0f / sample_time;
        encoder_speed_data.speed_rpm_lpf = ENCODER_SPEED_FILTER_ALPHA * encoder_speed_data.speed_rpm + (1.0f - ENCODER_SPEED_FILTER_ALPHA) * encoder_speed_data.speed_rpm_lpf;

        encoder_speed_data.accum_count = 0;
        encoder_speed_data.sample_div_cnt = 0;
    }

    // 保存本次计数值，供下次计算增量
    encoder_speed_data.last_count = current_count;
}


/**
 * @brief 获取当前电角度弧度值
 * @return 当前电角度，范围[0, 2π)
 */
float encoder_get_angle_rad(void)
{
    float elec_angle;

    // 读取当前编码器计数值并换算为电角度弧度值
    elec_angle = ((float)encoder_get_raw_count() / ENCODER_CPR) * MOTOR_POLE_PAIRS * ENCODER_TWO_PI;

    // 将电角度限制到[0, 2π)范围内
    while (elec_angle >= ENCODER_TWO_PI)
    {
        elec_angle -= ENCODER_TWO_PI;
    }

    return elec_angle;
}

/**
 * @brief 获取低通滤波后的转速
 * @return 最近一次更新得到的滤波转速，单位RPM
 */
float encoder_get_speed_rpm_lpf(void)
{
    // 返回最近一次更新得到的滤波转速
    return encoder_speed_data.speed_rpm_lpf;
}