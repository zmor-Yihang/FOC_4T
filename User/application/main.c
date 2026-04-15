#include "main.h"

int main(void)
{
    HAL_Init();
    clock_init();
    usart2_init();

    gpio_init();
    encoder_init();
    tim_init();
    adc_init();
    gpio_m1_enable();

    // current_closed_init(0.0f, 0.2f);
    speed_closed_init(2000);

    while (1)
    {
        // print_current_info();
        print_speed_info();
    }
}
