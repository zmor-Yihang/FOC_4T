#ifndef __FOC_H__
#define __FOC_H__

#include "stm32g4xx_hal.h"
#include "clark_park.h"
#include "svpwm.h"
#include "pid.h"
#include "tim.h"
#include "adc.h"
#include "motor_config.h"
#include "encoder.h"

// 对齐过程相关参数
#define FOC_ALIGN_D_AXIS_VOLTAGE      (0.3f)
#define FOC_ALIGN_SETTLE_TIME_MS      (500U)
#define FOC_ALIGN_SCAN_POINTS         (64U)
#define FOC_ALIGN_SCAN_REPEAT         (7U)
#define FOC_ALIGN_SAMPLE_INTERVAL_MS  (20U)

// SVPWM 电压矢量限幅比例, SVPWM 电压矢量限幅比例 (1/sqrt(3))
#define FOC_VOLTAGE_LIMIT_SVPWM_SCALE (0.57735026919f) 

/* FOC 核心控制对象 */
typedef struct
{
    abc_t i_abc; /* 电流采样值 */

    float target_speed; /* 目标值 */
    float target_id;
    float target_iq;

    float v_d_out; /* D轴电压输出 */
    float v_q_out; /* Q轴电压输出 */
    float i_q_out; /* Q轴电流输出 (速度环) */

    pid_controller_t *pid_id; /* PID控制器 */
    pid_controller_t *pid_iq;
    pid_controller_t *pid_speed;

    abc_t duty_cycle; /* 输出占空比 */

    float angle_offset; /* 编码器零点偏移 */

    float open_loop_angle_el; /* 开环运行角度 */
} foc_t;

/* FOC 控制函数 */
void foc_init(foc_t *handle, pid_controller_t *pid_id, pid_controller_t *pid_iq, pid_controller_t *pid_speed);
void foc_alignment(foc_t *handle);

/* 闭环控制 */
void foc_current_loop_run(foc_t *handle, dq_t i_dq, float angle_el);
void foc_speed_loop_run(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm, uint8_t speed_loop_divider);

/* 设置目标值 */
void foc_set_target_id(foc_t *handle, float id);
void foc_set_target_iq(foc_t *handle, float iq);
void foc_set_target_speed(foc_t *handle, float speed_rpm);


#endif /* __FOC_H__ */
