#include "as5600.h"

static uint8_t as5600_i2c_rx_buf[2] = {0};

/**
 * @brief 拼接I2C接收缓冲区得到原始计数 0-4095
 */
static inline uint16_t as5600_build_rawCount(const uint8_t *recv_buffer)
{
    return ((((uint16_t)recv_buffer[0]) << 8) | recv_buffer[1]) & 0x0FFF;
}

/**
 * @brief 发起一次原始角度异步读取
 */
static void as5600_startRead_rawCount(void)
{
    i2c_read_bytesAsync(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H, as5600_i2c_rx_buf, 2);
}

/**
 * @brief 初始化AS5600硬件接口
 */
void as5600_init(void)
{
    i2c_init();
    as5600_startRead_rawCount();
    HAL_Delay(1); // 确保首次I2C读写完成
}

/**
 * @brief 轮询获取最新原始角度计数
 * @retval 1: 获取到新数据; 0: 数据尚未准备好
 */
uint8_t as5600_poll_raw_count(uint16_t *raw_count)
{
    if (i2c_get_readState() == I2C_READ_STATE_BUSY)
    {
        return 0;
    }

    *raw_count = as5600_build_rawCount(as5600_i2c_rx_buf);
    as5600_startRead_rawCount(); // 消费完立即发起下一次读取

    return 1;
}
