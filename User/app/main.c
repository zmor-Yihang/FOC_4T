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

    motorControl_init(FOC_MODE_SPEED); // 统一闭环初始化
    motorControl_setSpeed(1000.0f);

    while (1)
    {
        motorControl_debugPrint();
    }
}
