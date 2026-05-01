#ifndef __FOC_H__
#define __FOC_H__

#include "stm32g4xx_hal.h"
#include "../alg/clark_park.h"
#include "../alg/pid.h"
#include "../app/user_config.h"

// 对齐过程相关参数
#define FOC_ALIGN_D_AXIS_VOLTAGE (0.3f)
#define FOC_ALIGN_SETTLE_TIME_MS (500U)
#define FOC_ALIGN_SCAN_POINTS (64)
#define FOC_ALIGN_SCAN_REPEAT (2U)
#define FOC_ALIGN_SAMPLE_INTERVAL_MS (20U)

// SVPWM 电压矢量限幅比例 (1/sqrt(3))
#define FOC_VOLTAGE_LIMIT_SVPWM_SCALE (0.57735026919f)

// 电流环执行周期
#define FOC_CURRENT_LOOP_DT_S (1.0f / FOC_CURRENT_LOOP_FREQ_HZ)

// 运行模式枚举
typedef enum
{
    FOC_MODE_CURRENT,
    FOC_MODE_SPEED,
    FOC_MODE_POSITION
} foc_mode_t;

// FOC 输入指令
typedef struct
{
    foc_mode_t mode;
    float target_position;
    float target_speed;
    float target_id;
    float target_iq;
} foc_cmd_t;

// FOC 状态反馈
typedef struct
{
    float angle_el;
    float speed_rpm;
    dq_t i_dq;
    float position_rad;
} foc_feedback_t;

/* FOC 内部运行状态 (用于调试) */
typedef struct
{
    float v_d_cmd;    /* D轴电压请求(限幅前) */
    float v_q_cmd;    /* Q轴电压请求(限幅前) */
    float v_d_out;    /* D轴实际电压输出(限幅后) */
    float v_q_out;    /* Q轴实际电压输出(限幅后) */
    float v_d_pi;     /* D轴PI输出 */
    float v_q_pi;     /* Q轴PI输出 */
    float v_d_ff;     /* D轴前馈输出 */
    float v_q_ff;     /* Q轴前馈输出 */
    abc_t duty_cycle; /* 输出占空比 */
} foc_state_t;

/* FOC 控制器集合 */
typedef struct
{
    pid_controller_t id;       /* d轴电流环 PID */
    pid_controller_t iq;       /* q轴电流环 PID */
    pid_controller_t speed;    /* 速度环 PID */
    pid_controller_t position; /* 位置环 PID */
} foc_controller_t;

/* FOC 核心控制对象 */
typedef struct
{
    foc_cmd_t cmd;
    foc_feedback_t feedback;
    foc_state_t state;
    foc_controller_t controller;

#if (FLUX_WEAK_ENABLE == 1)
    pid_controller_t flux_weak_pid;     // 弱磁 PI 控制器
    float flux_weak_u_current_filtered; // 弱磁电压幅值滤波值
#endif

    float angle_offset;        // 编码器零点偏移
    uint8_t speed_loop_cnt;    // 相对于电流环的分频执行速度环
    uint8_t position_loop_cnt; // 相对于速度环的分频执行位置环
} foc_t;

/* 初始化与校准 */
void foc_init(foc_t *handle);
void foc_alignment_zero(foc_t *handle);

/* 闭环统一入口 */
void foc_step(foc_t *handle, uint8_t speed_loop_divider, uint8_t position_loop_divider);

/* 设置目标值 */
void foc_set_cmd(foc_t *handle, const foc_cmd_t *cmd);

#endif /* __FOC_H__ */
