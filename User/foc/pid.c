#include "pid.h"

void pid_init(pid_controller_t *pid, pid_type_t type, float kp, float ki, float out_min, float out_max)
{
    pid->type = type;
    pid->kp = kp;
    pid->ki = ki;

    pid->error = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;

    // kt = ki，Microchip AN1078 笔记的取法
    pid->kt = ki;
    pid->backcalc_error = 0.0f;

    // 积分限幅要足够大，不要限制 back-calc 的收敛
    pid->integral_max = fmaxf(fabsf(out_min), fabsf(out_max));
}

float pid_calculate(pid_controller_t *pid, float setpoint, float feedback)
{
    pid->error = setpoint - feedback; // 误差

    float p_term = pid->kp * pid->error; // 比例项

    // back-calc 自动调节
    float integral_increment = pid->kp * pid->ki * pid->error - pid->kt * pid->backcalc_error; // 该周期的积分项 + back-calc 修正

    pid->integral += integral_increment; // 积分项累加

    // 积分限幅，同时要足够大以允许 back-calc 收敛
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    float out_unclamped = p_term + pid->integral; // 未限幅的PI输出

    // 如果是电流环PI，直接输出未限幅的值，由外部负责限幅，因为电流环输出是电压，需要做矢量限幅
    if (pid->type == PID_TYPE_CURRENT)
    {
        pid->out = out_unclamped;
        return pid->out;
    }

    // 速度环输出限幅
    if (out_unclamped > pid->out_max)
        pid->out = pid->out_max;
    else if (out_unclamped < pid->out_min)
        pid->out = pid->out_min;
    else
        pid->out = out_unclamped;

    pid->backcalc_error = out_unclamped - pid->out; // 饱和误差，用于下一周期的 back-calc 修正

    return pid->out;
}

// PI复位
void pid_reset(pid_controller_t *pid)
{
    pid->error = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;
    pid->backcalc_error = 0.0f;
}
