#include "current_sense.h"

static inline float current_sense_raw_to_voltage(uint16_t adc_raw)
{
    return (float)adc_raw * ADC_VREF / ADC_RESOLUTION;
}

void current_sense_convert_raw(const adc_raw_values_t *raw, const current_sense_offset_t *offsets, abc_t *currents)
{
    float vout_a = current_sense_raw_to_voltage(raw->ia_raw); // A相采样电压
    float vout_b = current_sense_raw_to_voltage(raw->ib_raw); // B相采样电压
    float ia_hw = (vout_a - CURRENT_SENSE_REF_VOLTAGE - offsets->ia_offset) * CURRENT_SENSE_SCALE;
    float ib_hw = (vout_b - CURRENT_SENSE_REF_VOLTAGE - offsets->ib_offset) * CURRENT_SENSE_SCALE;
    float ic_hw = -(ia_hw + ib_hw);

#if (MOTOR_PHASE_SWAP == 0)
    currents->a = ia_hw;
    currents->b = ib_hw;
    currents->c = ic_hw;
#else
    currents->a = ib_hw;
    currents->b = ia_hw;
    currents->c = ic_hw;
#endif
}

void current_sense_dbg_get_regular_abc(abc_t *currents)
{
    adc_raw_values_t raw;
    current_sense_offset_t offsets;

    adcDebug_get_regularRaw(&raw);
    adcDebug_get_offset(&offsets);
    current_sense_convert_raw(&raw, &offsets, currents);
}

void current_sense_get_injected_abc(abc_t *currents)
{
    adc_raw_values_t raw;
    current_sense_offset_t offsets;

    adc_get_injectedRaw(&raw);
    adcDebug_get_offset(&offsets);
    current_sense_convert_raw(&raw, &offsets, currents);
}

void current_sense_dbg_get_offset(current_sense_offset_t *offsets)
{
    adcDebug_get_offset(offsets);
}
