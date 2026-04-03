#include "main.h"

int main(void)
{
    HAL_Init();
    clock_init();
    usart2_init();
    // led1_init();
    // key_init();
    // led2_init();
    // as5047_init();
    // tim3_init();
    // tim1_init();
    // adc1_init();

    // speed_closed_init(8000.0f);
    // flux_weak_speed_closed_init(8000.0f);

    while (1)
    {

        printf("hello world\r\n");
        // print_speed_info();
        // print_flux_weak_speed_info();
    }
}
