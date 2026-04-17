#include "main.h"

int main(void)
{
    HAL_Init();
    clock_init();
    usart_init();

    gpio_init();
    encoder_init();
    tim_init();
    adc_init();
    gpio_m1_enable();

    // currentClosed_init(0.0f, 0.5f);
    speedClosed_init(5);

    while (1)
    {
        // currentClosedDebug_print_info();
        speedClosedDebug_print_info();
    }
}
