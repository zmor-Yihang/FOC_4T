#include "i2c.h"

I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;

volatile int a = 0;
volatile int i2c3_err_cnt = 0;

void i2c_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE(); // DMA请求路由器
    __HAL_RCC_DMA1_CLK_ENABLE();    // DMA1_Channel4 (I2C3_RX)
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
    hi2c3.Init.Timing = 0x00702991;                       // 时序参数: 400kHz?
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  // 7位地址模式
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // 单地址模式
    hi2c3.Init.OwnAddress1 = 0;                           // 自身地址
    hi2c3.Init.OwnAddress2 = 0;                           // 第二地址
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;         // 地址掩码
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // 禁止广播呼叫

    HAL_I2C_Init(&hi2c3);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0);

    hdma_i2c3_rx.Instance = DMA1_Channel4;
    hdma_i2c3_rx.Init.Request = DMA_REQUEST_I2C3_RX;             // I2C3_RX作为DMA请求源
    hdma_i2c3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;          // 外设 -> 内存 方向
    hdma_i2c3_rx.Init.PeriphInc = DMA_PINC_DISABLE;              // 外设地址不自增
    hdma_i2c3_rx.Init.MemInc = DMA_MINC_ENABLE;                  // 内存地址自增
    hdma_i2c3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; // 外设字节对齐 (8bit)
    hdma_i2c3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    // 内存字节对齐 (8bit)
    hdma_i2c3_rx.Init.Mode = DMA_NORMAL;                         // 正常模式
    hdma_i2c3_rx.Init.Priority = DMA_PRIORITY_HIGH;              // 高优先级
    HAL_DMA_Init(&hdma_i2c3_rx);

    __HAL_LINKDMA(&hi2c3, hdmarx, hdma_i2c3_rx); // 将DMA句柄绑定到I2C3 RX

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 2, 0); // 与串口DMA一致, 避免抢占FOC关键中断
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);         // 使能DMA中断

    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
}

void i2c_write_bytes(uint16_t dev_addr, uint16_t reg, uint8_t *send_buffer, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, send_buffer, len, 1);
}

void i2c_read_bytes(uint16_t dev_addr, uint16_t reg, uint8_t *recv_buffer, uint8_t len)
{
    if (HAL_I2C_Mem_Read_DMA(&hi2c3, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, recv_buffer, len) != HAL_OK)
    {
        i2c3_err_cnt++;
    }
}

void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_i2c3_rx);
}

void I2C3_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c3);
}

void I2C3_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c3);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C3)
    {
        a++;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C3)
    {
        i2c3_err_cnt++;
    }
}