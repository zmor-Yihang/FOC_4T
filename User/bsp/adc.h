#ifndef __ADC_H
#define __ADC_H

#include "stm32g4xx_hal.h"
#include "delay.h"

#define ADC_REF_VOLTAGE 1.65f                                     // INA240 输出零点 (REF1=VS, REF2=GND → VS/2)
#define ADC_SHUNT_RES 0.01f                                       // 分流电阻 (Ω)
#define ADC_INA_GAIN 50.0f                                        // INA240A2 增益 (V/V)
#define ADC_CURRENT_SCALE (1.0f / (ADC_INA_GAIN * ADC_SHUNT_RES)) // = 2.0 A/V

#define ADC_VREF 3.3f                                             // ADC 参考电压
#define ADC_RESOLUTION 4096.0f                                    // 12位 ADC 分辨率

#define ADC_CALIB_SAMPLES 256  // 零点标定采样次数
#define ADC_CALIB_DELAY_US 20  // 每次标定采样间隔 (us)
#define ADC_CALIB_SETTLE_MS 20 // 标定前等待模拟前端稳定 (ms)

// 三相电流值
typedef struct
{
    float ia;
    float ib;
    float ic;
} adc_values_t;

// AB相零点偏移
typedef struct
{
    float ia_offset;
    float ib_offset;
} adc_offset_t;

// 注入组中断回调函数指针类型
typedef void (*adc_injected_callback_p)(void);

void adc_init(void);
void adc_get_offset(adc_offset_t *offsets);
void adc_get_regular_values(adc_values_t *values);
void adc_get_injected_values(adc_values_t *values);
uint32_t adc_get_injected_irq_count(void);
void adc_register_injected_callback(adc_injected_callback_p callback);

#endif /* __ADC_H */
