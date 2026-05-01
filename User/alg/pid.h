#ifndef __PID__H__ 
#define __PID__H__

#include "math.h"
#include "../utils/utils.h"

typedef enum
{
    PID_ROLE_CURRENT = 0,
    PID_ROLE_SPEED,
    PID_ROLE_POSITION
} pid_role_t;

typedef enum
{
    PID_MODE_P = 0,
    PID_MODE_PI,
    PID_MODE_PID
} pid_mode_t;

/* P/PI/PID 控制器结构体 */
typedef struct
{
    pid_role_t role;    /* 控制器所属环路 */
    pid_mode_t mode;    /* 控制律模式 */
    float kp;           /* 比例系数 */
    float ki;           /* 积分系数 */
    float kd;           /* 微分系数 */

    float error;        /* 当前误差 */
    float prev_error;   /* 上一拍误差 */
    float integral;     /* 积分项累加 */
    float derivative;   /* 微分项 */

    float out;           /* 控制器输出 */
    float out_min;       /* 输出下限 */
    float out_max;       /* 输出上限 */
    float integral_max;  /* 积分限幅 */

    float kt;            /* Back-calculation 增益 */
    float backcalc_error;/* 上一拍饱和误差 */
} pid_controller_t;

/* P/PI/PID 控制器初始化 */
void pid_init(pid_controller_t *pid, pid_role_t role, pid_mode_t mode, float kp, float ki, float kd, float out_min, float out_max);

/* 控制器计算，dt单位：秒 */
float pid_calculate(pid_controller_t *pid, float setpoint, float feedback, float dt);

/* 控制器复位 */
void pid_reset(pid_controller_t *pid);

#endif /* __PID__H__ */
