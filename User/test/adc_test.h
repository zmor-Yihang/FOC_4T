#ifndef __ADC_TEST_H__
#define __ADC_TEST_H__

#include "stm32g4xx_hal.h"
#include <stdio.h>
#include "../bsp/adc.h"
#include "../sensor/current_sense.h"
#include "../utils/print.h"

void adc_test_init(void);
void adc_test_poll(void);

#endif /* __ADC_TEST_H__ */