#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#define U_DC 12.0 // 直流母线电压

#define MOTOR_POLE_PAIRS 7  // 电机极对数
#define MOTOR_RS_Ω 2.3f     // 定子电阻，实测1欧
#define MOTOR_LD_H 0.86e-3f // d轴电感(H)
#define MOTOR_LQ_H 0.86e-3f // q轴电感(H)
#define MOTOR_PSI_F 0.0035f // 永磁体磁链(Wb)
#define MOTOR_PHASE_SWAP 1  // 1：启用相序翻转

#define FOC_CURRENT_LOOP_FREQ_HZ 10000.0f // 电流环执行频率(Hz)
#define FOC_SPEED_LOOP_DIVIDER 10U        // 速度环相对电流环的分频系数
#define FOC_DECOUPLING_ENABLE 1           // 1:启用前馈解耦 0:关闭前馈解耦
#define FOC_ELEC_ANGLE_TRIM_RAD 0.0f      // 电角度手动微调量(rad)，正常情况下保持为0，仅在排查固定偏差时临时微调

#define ENCODER_COUNT_SWAP 0                // 1：启用编码器计数翻转
#define ENCODER_PLL_KP 1776.0f              // 编码器速度 PLL 比例增益
#define ENCODER_PLL_KI 1.58e6f              // 编码器速度 PLL 积分增益
#define ENCODER_PLL_SPEED_LIMIT_RPM 3000.0f // 编码器 PLL 机械转速限幅(rpm)

#endif /* __USER_CONFIG_H__ */
