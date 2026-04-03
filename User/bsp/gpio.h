#ifndef __GPIO_H__
#define __GPIO_H__

// PA4  PB4  Mosfet驱动使能引脚
#include "stm32g4xx_hal.h"

#define MOSFET_ENA_PORT GPIOA
#define MOSFET_ENA_PIN GPIO_PIN_4

#define MOSFET_ENB_PORT GPIOB
#define MOSFET_ENB_PIN GPIO_PIN_4

void gpio_mosfet_init(void);
void gpio_mosfet_j1_enable(void);
void gpio_mosfet_j2_enable(void);
void gpio_mosfet_j1_disable(void);
void gpio_mosfet_j2_disable(void);



#endif /* __GPIO_H__ */
