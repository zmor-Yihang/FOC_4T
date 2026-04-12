#include "main.h"

int main(void)
{
    HAL_Init();
    clock_init();
    usart2_init();

    gpio_init();
    gpio_m1_enable();
    encoder_init();
    tim_init();
    adc_init();

    current_closed_init(0.0f, 0.2f);
    // speed_closed_init(2000);

    while (1)
    {
        printf("a = %d，i2c3_err_cnt = %d\r\n", a, i2c3_err_cnt);
        // print_current_info();
        // print_speed_info();
    }
}
