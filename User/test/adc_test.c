#include "adc_test.h"

#include <stdio.h>

#include "adc.h"
#include "print.h"

void adc_test_init(void)
{
    adc_init();
}

void adc_test_poll(void)
{
    adc_values_t adc_values;
    adc_offset_t adc_offset;
    float debug_data[5];

    adc_get_regular_values(&adc_values);
    adc_get_offset(&adc_offset);

    debug_data[0] = adc_values.ia;
    debug_data[1] = adc_values.ib;
    debug_data[2] = adc_values.ic;
    debug_data[3] = adc_offset.ia_offset;
    debug_data[4] = adc_offset.ib_offset;

    printf_vofa(debug_data, 5);

    printf_period(200,
                  "adc ia=%.4fA, ib=%.4fA, ic=%.4fA, off_a=%.5fV, off_b=%.5fV\r\n",
                  adc_values.ia,
                  adc_values.ib,
                  adc_values.ic,
                  adc_offset.ia_offset,
                  adc_offset.ib_offset);
}