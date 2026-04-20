#include "main.h"

int main(void)
{
    // 基础外设初始化
    HAL_Init();
    clock_init();
    usart_init();

    gpio_init();
    encoder_init();
    tim_init();
    adc_init();
    gpio_m1_enable();

    // 运行模式三选一（保持只启用一个）
    // currentClosed_init(0.0f, 0.5f);
    speedClosed_init(3000);
    // 无感速度闭环：角度与速度由磁链观测器提供
    // fluxObseverClosed_init(30);

    while (1)
    {
        // 调试数据输出三选一（与上面模式对应）
        // currentClosedDebug_print_info();
        speedClosedDebug_print_info();
        // fluxObseverClosedDebug_print_info();
    }
}
