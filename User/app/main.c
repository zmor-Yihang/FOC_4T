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

    foc_cmd_t motor_cmd = {
        .target_speed = 1000.0f,
    };

    motorControl_init(FOC_MODE_SPEED, &motor_cmd); // 统一闭环初始化

    while (1)
    {
        motorControl_debugPrint();
    }
}
