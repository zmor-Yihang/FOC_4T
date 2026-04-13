#include "i2c.h"

I2C_HandleTypeDef hi2c3;

static volatile uint8_t i2c3_read_busy = 0U;
static volatile uint8_t i2c3_read_done = 0U;

void i2c_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    __HAL_RCC_I2C3_CLK_ENABLE();

    // PC8: I2C3_SCL, PC9: I2C3_SDA
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    hi2c3.Instance = I2C3;                                // 选择I2C3
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;     // 不使能无拉伸模式, 即允许时钟拉伸
    hi2c3.Init.Timing = 0x00802172;                       // 时序参数: 400kHz
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  // 7位地址模式
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // 单地址模式
    hi2c3.Init.OwnAddress1 = 0;                           // 自身地址
    hi2c3.Init.OwnAddress2 = 0;                           // 第二地址
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;         // 地址掩码
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // 禁止广播呼叫

    HAL_I2C_Init(&hi2c3);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0);

    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
}

void i2c_read_bytes(uint16_t dev_addr, uint16_t reg, uint8_t *recv_buffer, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, recv_buffer, len, 1);
}

HAL_StatusTypeDef i2c_read_bytes_it(uint16_t dev_addr, uint16_t reg, uint8_t *recv_buffer, uint8_t len)
{
    HAL_StatusTypeDef status;

    if (i2c3_read_busy != 0U)
    {
        return HAL_BUSY;
    }

    i2c3_read_done = 0U;
    status = HAL_I2C_Mem_Read_IT(&hi2c3, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, recv_buffer, len);

    if (status == HAL_OK)
    {
        i2c3_read_busy = 1U;
    }

    return status;
}

uint8_t i2c_read_is_busy(void)
{
    return i2c3_read_busy;
}

uint8_t i2c_read_is_done(void)
{
    return i2c3_read_done;
}

void i2c_read_clear_done_flag(void)
{
    i2c3_read_done = 0U;
}

void I2C3_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c3);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C3)
    {
        i2c3_read_busy = 0U;
        i2c3_read_done = 1U;
    }
}
