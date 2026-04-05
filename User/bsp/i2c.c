#include "i2c.h"

I2C_HandleTypeDef hi2c3;

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
    hi2c3.Init.Timing = 0x00702991;                       // 时序参数: 400kHz, 上升下降时间: 100ns
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  // 7位地址模式
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // 单地址模式
    hi2c3.Init.OwnAddress1 = 0;                           // 自身地址
    hi2c3.Init.OwnAddress2 = 0;                           // 第二地址
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;         // 地址掩码
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // 禁止广播呼叫

    HAL_I2C_Init(&hi2c3);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0);
}

void i2c_write_bytes(uint16_t dev_addr, uint16_t reg, uint8_t *send_buffer, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, send_buffer, len, 100);
}

void i2c_read_bytes(uint16_t dev_addr, uint16_t reg, uint8_t *recv_buffer, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, recv_buffer, len, 100);
}

void i2c_scan_bus(void)
{
    HAL_StatusTypeDef ret;
    uint8_t addr = 0x36;

    printf("\r\n[I2C] probing 0x%02X...\r\n", addr);

    ret = HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(addr << 1), 5, 50);

    if (ret == HAL_OK)
    {
        printf("[I2C] found device at 0x%02X\r\n", addr);
    }
    else
    {
        printf("[I2C] device 0x%02X not found\r\n", addr);
        printf("[I2C] hi2c3.ErrorCode = 0x%08lX\r\n", hi2c3.ErrorCode);
    }
}