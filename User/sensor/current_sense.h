#ifndef __CURRENT_SENSE_H__
#define __CURRENT_SENSE_H__

#include "../bsp/adc.h"
#include "../alg/clark_park.h"
#include "../app/user_config.h"

#define CURRENT_SENSE_REF_VOLTAGE 1.65f                                                 // 电流采样参考电压(V)
#define CURRENT_SENSE_SHUNT_RES 0.01f                                                   // 分流电阻(Ω)
#define CURRENT_SENSE_AMP_GAIN 50.0f                                                    // 电流采样运放增益(V/V)
#define CURRENT_SENSE_SCALE (1.0f / (CURRENT_SENSE_AMP_GAIN * CURRENT_SENSE_SHUNT_RES)) // 电流换算系数(A/V)

typedef adc_offset_t current_sense_offset_t;

abc_t currentSense_get_injectedValue(void);
abc_t currentSenseDebug_get_regularValue(void);
current_sense_offset_t currentSense_get_offset(void);

#endif /* __CURRENT_SENSE_H__ */
