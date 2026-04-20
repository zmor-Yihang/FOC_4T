#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

// 直流母线电压
#define U_DC                             12.0

// 数学常量
#define MATH_PI                          3.14159265358979323846f // 圆周率 PI
#define MATH_TWO_PI                      (2.0f * MATH_PI)        // 2*PI
#define MATH_INV_SQRT3                   0.57735026919f          // 1/sqrt(3)
#define MATH_SQRT3_BY_2                  0.86602540378f          // sqrt(3)/2

// 电机参数
#define MOTOR_POLE_PAIRS                 7        // 电机极对数
#define MOTOR_RS_Ω                       1.2f     // 定子电阻，实测1欧
#define MOTOR_LD_H                       0.86e-3f // d轴电感(H)
#define MOTOR_LQ_H                       0.86e-3f // q轴电感(H)
#define MOTOR_PSI_F                      0.0033f  // 永磁体磁链(Wb)
#define MOTOR_PHASE_SWAP                 1        // 1：启用相序翻转

// 控制参数
#define FOC_CURRENT_LOOP_FREQ_HZ         10000.0f // 电流环执行频率(Hz)
#define FOC_SPEED_LOOP_DIVIDER           10U      // 速度环相对电流环的分频系数
#define FOC_DECOUPLING_ENABLE            1        // 1:启用前馈解耦 0:关闭前馈解耦
#define FOC_ELEC_ANGLE_TRIM_RAD          0.0f     // 电角度手动微调量(rad)，正常情况下保持为0，仅在排查固定偏差时临时微调

// 编码器参数
#define ENCODER_COUNT_SWAP               0        // 1：启用编码器计数翻转
#define ENCODER_PLL_KP                   1776.0f  // 编码器速度 PLL 比例增益
#define ENCODER_PLL_KI                   1.58e6f  // 编码器速度 PLL 积分增益
#define ENCODER_PLL_SPEED_LIMIT_RPM      3000.0f  // 编码器 PLL 机械转速限幅(rpm)

// 无感磁链观测器参数
#define FLUX_OBSERVER_GAMMA               3.0e7f                           // 观测器非线性增益
#define FLUX_OBSERVER_TS_S                (1.0 / FOC_CURRENT_LOOP_FREQ_HZ) // 观测器执行周期(s)
#define FLUX_OBSERVER_PLL_KP              1776.0f                          // PLL比例增益
#define FLUX_OBSERVER_PLL_KI              1.58e6f                         // PLL积分增益
#define FLUX_OBSERVER_PLL_SPEED_LIMIT_RPM 5000.0f                          // 机械转速限幅(rpm)


// 数据类型
typedef struct
{
    float a;
    float b;
    float c;
} abc_t; // 三相坐标系

typedef struct
{
    float alpha;
    float beta;
} alphabeta_t; // 两相静止坐标系

typedef struct
{
    float d;
    float q;
} dq_t; // 两相旋转坐标系

#endif /* __USER_CONFIG_H__ */
