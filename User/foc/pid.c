#include "pid.h"

// PI控制器初始化
void pid_init(pid_controller_t *pid, float kp, float ki, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;

    pid->error = 0.0f;
    pid->integral = 0.0f;

    pid->out = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;

    // 积分限幅取输出范围的75%
    pid->integral_max = 0.75f * fmaxf(fabsf(out_min), fabsf(out_max));

    // Back-calculation 增益: Kt = 1/Kp，将输出饱和误差折算回积分器
    // kp=0 时为纯积分控制器，禁用 Back-calculation
    pid->kt = (kp != 0.0f) ? (1.0f / kp) : 0.0f;
    pid->backcalc_error = 0.0f;
}

/**
 * @brief PI计算
 * @param pid: PI控制器结构体指针
 * @param setpoint: 设定值
 * @param feedback: 反馈值
 * @return [out_min, out_max]
 * 
 */
float pid_calculate(pid_controller_t *pid, float setpoint, float feedback)
{
    // 计算当前误差
    pid->error = setpoint - feedback;

    // 比例项
    float p_term = pid->kp * pid->error;

    // 积分项增量（包含上一拍的 Back-calculation 反馈校正）
    // Kt = 1/Kp，用于将输出饱和误差折算回积分器
    float integral_increment = (pid->ki * pid->kp) * pid->error + pid->kt * pid->backcalc_error;

    // 更新积分
    pid->integral += integral_increment;

    // 积分限幅
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    // 计算未饱和输出
    float out_unclamped = p_term + pid->integral;

    // 输出限幅
    if (out_unclamped > pid->out_max)
        pid->out = pid->out_max;
    else if (out_unclamped < pid->out_min)
        pid->out = pid->out_min;
    else
        pid->out = out_unclamped;

    // Back-calculation: 记录饱和误差，供下一拍修正积分
    // backcalc_error = u_clamped - u_unclamped
    pid->backcalc_error = pid->out - out_unclamped;

    return pid->out;
}

// PI复位
// pid: PI控制器结构体指针
void pid_reset(pid_controller_t *pid)
{
    pid->error = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;
    pid->backcalc_error = 0.0f;
}
