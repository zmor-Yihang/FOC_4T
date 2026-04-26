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

    // currentClosed_init(0.0f, 0.5f);
    // speedClosed_init(10); // 速度闭环
    speedWeakClosed_init(1500);// 弱磁速度闭环
    // fluxObseverClosed_init(2000);// 无感速度闭环

    while (1)
    {
        // currentClosedDebug_print_info();
        // speedClosedDebug_print_info();
        speedWeakClosedDebug_print_info();
        // fluxObseverClosedDebug_print_info();
    }
}
