#ifndef __CURRENT_SENSE_H__
#define __CURRENT_SENSE_H__

#include "adc.h"
#include "clark_park.h"
#include "motor_config.h"

#define CURRENT_SENSE_REF_VOLTAGE 1.65f                                                 // 电流采样参考电压(V)
#define CURRENT_SENSE_SHUNT_RES 0.01f                                                   // 分流电阻(Ω)
#define CURRENT_SENSE_AMP_GAIN 50.0f                                                    // 电流采样运放增益(V/V)
#define CURRENT_SENSE_SCALE (1.0f / (CURRENT_SENSE_AMP_GAIN * CURRENT_SENSE_SHUNT_RES)) // 电流换算系数(A/V)

typedef adc_offset_t current_sense_offset_t;

void current_sense_convert_raw(const adc_raw_values_t *raw, const current_sense_offset_t *offsets, abc_t *currents);
void current_sense_get_injected_abc(abc_t *currents);
void current_sense_dbg_get_regular_abc(abc_t *currents);
void current_sense_dbg_get_offset(current_sense_offset_t *offsets);

#endif /* __CURRENT_SENSE_H__ */
