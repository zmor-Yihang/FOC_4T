#include "main.h"
#include <stdio.h>

#include "usart.h"
#include "gpio.h"
#include "clock.h"

#include "sensorless_luenberger.h"
#include "if_open.h"
#include "current_closed.h"
#include "speed_closed.h"
#include "sensorless_smo.h"
#include "flux_weak_speed_closed.h"
#include "speed_closed_with_smo.h"
#include "speed_closed_with_luenberger.h"
#include "speed_closed_with_flux_observer.h"
#include "sensorless_flux_observer.h"
#include "sensorless_flux_observer.h"

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
    // adc_init();

    // speed_closed_init(8000.0f);
    // flux_weak_speed_closed_init(8000.0f);

    while (1)
    {

        printf("hello world\r\n");
        // print_speed_info();
        // print_flux_weak_speed_info();
    }
}
