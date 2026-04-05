#include "tim.h"

TIM_HandleTypeDef htim2; // TIM2句柄
TIM_HandleTypeDef htim3; // TIM3句柄
TIM_HandleTypeDef htim6; // TIM6句柄: 速度环定时器

void tim_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_init_struct = {0};
    TIM_MasterConfigTypeDef master_config = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE(); // 使能GPIOA时钟
    __HAL_RCC_TIM2_CLK_ENABLE();  // 使能TIM2时钟（A相PWM + ADC触发）
    __HAL_RCC_TIM3_CLK_ENABLE();  // 使能TIM3时钟（B相/C相PWM）

    // PA5 → TIM2_CH1 (A相)
    gpio_init_struct.Pin = GPIO_PIN_5;                  // 选择PA5引脚
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;            // 复用推挽输出模式
    gpio_init_struct.Pull = GPIO_NOPULL;                // 无上下拉
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 最高GPIO翻转速度
    gpio_init_struct.Alternate = GPIO_AF1_TIM2;         // 复用功能映射到TIM2
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);            // 按上述配置初始化PA5

    // PA6/PA7 → TIM3_CH1/CH2 (B相/C相)
    gpio_init_struct.Pin = GPIO_PIN_6 | GPIO_PIN_7; // 选择PA6和PA7引脚
    gpio_init_struct.Alternate = GPIO_AF2_TIM3;     // 复用功能映射到TIM3
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);        // 按上述配置初始化PA6/PA7

    htim2.Instance = TIM2;                                        // 绑定TIM2外设
    htim2.Init.Prescaler = TIM1_PRESCALER;                        // 预分频值（PSC）
    htim2.Init.Period = TIM1_PERIOD;                              // 自动重装载值（ARR），决定PWM频率
    htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;      // 中央对齐模式1（向上/向下都计数，产生对称PWM）
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;            // 不分频
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // 使能ARR预装载
    HAL_TIM_PWM_Init(&htim2);                                     // 初始化TIM2为PWM模式

    master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;         // 更新事件(计数器下溢)作为TRGO输出触发ADC
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // 不需要主从级联
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &master_config);

    htim3.Instance = TIM3;                                        // 绑定TIM3外设
    htim3.Init.Prescaler = TIM1_PRESCALER;                        // 预分频值，与TIM2保持一致
    htim3.Init.Period = TIM1_PERIOD;                              // 自动重装载值，与TIM2保持一致
    htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;      // 中央对齐模式1
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;            // 不分频
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // 使能ARR预装载
    HAL_TIM_PWM_Init(&htim3);                                     // 初始化TIM3为PWM模式

    tim_oc_init_struct.OCMode = TIM_OCMODE_PWM1;         // PWM模式1：CNT < CCR时输出有效电平
    tim_oc_init_struct.OCPolarity = TIM_OCPOLARITY_HIGH; // 有效电平为高电平
    tim_oc_init_struct.OCFastMode = TIM_OCFAST_DISABLE;  // 禁用快速模式
    tim_oc_init_struct.Pulse = TIM1_PERIOD / 2;          // 初始占空比50%（CCR值）

    HAL_TIM_PWM_ConfigChannel(&htim2, &tim_oc_init_struct, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &tim_oc_init_struct, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &tim_oc_init_struct, TIM_CHANNEL_2);

    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // 启动B相PWM输出
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // 启动C相PWM输出
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 启动A相PWM输出
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
