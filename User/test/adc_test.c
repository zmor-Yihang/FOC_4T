#include "adc_test.h"

#include <stdio.h>

#include "adc.h"
#include "sensor/current_sense.h"
#include "print.h"

void adc_test_init(void)
{
    adc_init();
}

void adc_test_poll(void)
{
    abc_t phase_current;
    current_sense_offset_t current_offset;
    float debug_data[5];

    current_sense_dbg_get_regular_abc(&phase_current);
    current_sense_dbg_get_offset(&current_offset);

    debug_data[0] = phase_current.a;
    debug_data[1] = phase_current.b;
    debug_data[2] = phase_current.c;
    debug_data[3] = current_offset.ia_offset;
    debug_data[4] = current_offset.ib_offset;

    vofa_send(debug_data, 5);
}