#ifndef __FOC_H__
#define __FOC_H__

#include "stm32g4xx_hal.h"
#include "../alg/clark_park.h"
#include "../alg/pid.h"
#include "../app/user_config.h"

// 对齐过程相关参数
#define FOC_ALIGN_D_AXIS_VOLTAGE (0.5f)
#define FOC_ALIGN_SETTLE_TIME_MS (500U)
#define FOC_ALIGN_SCAN_POINTS (64)
#define FOC_ALIGN_SCAN_REPEAT (4U)
#define FOC_ALIGN_SAMPLE_INTERVAL_MS (10U)

// SVPWM 电压矢量限幅比例 (1/sqrt(3))
#define FOC_VOLTAGE_LIMIT_SVPWM_SCALE (0.57735026919f)

// 电流环执行周期
#define FOC_CURRENT_LOOP_DT_S (1.0f / FOC_CURRENT_LOOP_FREQ_HZ)

/* FOC 核心控制对象 */
typedef struct
{
    float target_speed; /* 目标值 */
    float target_id;
    float target_iq;

    float v_d_out; /* D轴电压输出 */
    float v_q_out; /* Q轴电压输出 */
    float v_d_pi;  /* D轴PI输出 */
    float v_q_pi;  /* Q轴PI输出 */
    float v_d_ff;  /* D轴前馈输出 */
    float v_q_ff;  /* Q轴前馈输出 */

    pid_controller_t *pid_id; /* PID控制器 */
    pid_controller_t *pid_iq;
    pid_controller_t *pid_speed;

    abc_t duty_cycle; /* 输出占空比 */

    float angle_offset; /* 编码器零点偏移 */
} foc_t;

/* 初始化与校准 */
void foc_init(foc_t *handle, pid_controller_t *pid_id, pid_controller_t *pid_iq, pid_controller_t *pid_speed);
void zero_alignment(foc_t *handle);

/* 闭环控制 */
void loopControl_run_currentLoop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm);
void loopControl_run_speedLoop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm, uint8_t speed_loop_divider);

/* 设置目标值 */
void foc_set_id(foc_t *handle, float id);
void foc_set_iq(foc_t *handle, float iq);
void foc_set_speed(foc_t *handle, float speed_rpm);

#endif /* __FOC_H__ */

