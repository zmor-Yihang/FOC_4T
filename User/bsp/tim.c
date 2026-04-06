#include "tim.h"

TIM_HandleTypeDef htim2; // TIM2句柄
TIM_HandleTypeDef htim3; // TIM3句柄

void tim_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_init_struct = {0};
    TIM_MasterConfigTypeDef master_config = {0};
    TIM_SlaveConfigTypeDef slave_config = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    // GPIO配置
    gpio_init_struct.Pin = GPIO_PIN_5;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    gpio_init_struct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio_init_struct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    // TIM2: Master
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = TIM1_PRESCALER;                        // 预分频
    htim2.Init.Period = TIM1_PERIOD;                              // 自动重装载值
    htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;      // 中心对齐模式1
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;            // 时钟分频
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // 使能自动重装载预装载
    HAL_TIM_PWM_Init(&htim2);

    // TIM2输出TRGO
    master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;        // 触发adc和tim3
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE; // 使能主从模式
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &master_config);

    // TIM3: Slave
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = TIM1_PRESCALER;                        // 预分频
    htim3.Init.Period = TIM1_PERIOD;                              // 自动重装载值
    htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;      // 中心对齐模式1
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;            // 时钟分频
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // 使能自动重装载预装载
    HAL_TIM_PWM_Init(&htim3);

    // TIM3接收TIM2的触发
    slave_config.SlaveMode = TIM_SLAVEMODE_TRIGGER; // 触发模式
    slave_config.InputTrigger = TIM_TS_ITR1;        // 芯片手册确认
    HAL_TIM_SlaveConfigSynchro(&htim3, &slave_config);

    // PWM通道配置
    tim_oc_init_struct.OCMode = TIM_OCMODE_PWM1;         // PWM模式1
    tim_oc_init_struct.OCPolarity = TIM_OCPOLARITY_HIGH; // 输出极性高
    tim_oc_init_struct.OCFastMode = TIM_OCFAST_DISABLE;  // 关闭快速模式
    tim_oc_init_struct.Pulse = TIM1_PERIOD / 2;

    HAL_TIM_PWM_ConfigChannel(&htim2, &tim_oc_init_struct, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &tim_oc_init_struct, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &tim_oc_init_struct, TIM_CHANNEL_2);

    // 先启动从定时器通道，但不会真正跑，等主触发
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // 最后启动主定时器，TIM2一启动，TIM3同步启动
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void tim_set_pwm_duty(float duty1, float duty2, float duty3)
{
    uint32_t compare1 = (uint32_t)(duty1 * TIM1_PERIOD);
    uint32_t compare2 = (uint32_t)(duty2 * TIM1_PERIOD);
    uint32_t compare3 = (uint32_t)(duty3 * TIM1_PERIOD);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare1); // 写入A相CCR（PA5/TIM2_CH1）
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare2); // 写入B相CCR（PA6/TIM3_CH1）
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare3); // 写入C相CCR（PA7/TIM3_CH2）
}
