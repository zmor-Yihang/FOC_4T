#include "main.h"

int main(void)
{
    // 外设初始化
    HAL_Init();
    clock_init();
    usart_init();

    gpio_init();
    encoder_init();
    tim_init();
    adc_init();
    gpio_m1_enable();

    // 运行模式三选一
    // currentClosed_init(0.0f, 0.5f);
    speedClosed_init(3000);
    // 无感速度闭环：非线性磁链观测器
    // fluxObseverClosed_init(30);

    while (1)
    {
        // 调试数据输出三选一
        // currentClosedDebug_print_info();
        speedClosedDebug_print_info();
        // fluxObseverClosedDebug_print_info();
    }
}
