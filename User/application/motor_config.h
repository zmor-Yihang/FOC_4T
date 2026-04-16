#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

#define U_DC 12.0          // 直流母线电压
#define MOTOR_POLE_PAIRS 7 // 电机极对数

#define MOTOR_RS_Ω 2.3f     // 定子电阻，实测1欧
#define MOTOR_LD_H 0.86e-3f // d轴电感(H)
#define MOTOR_LQ_H 0.86e-3f // q轴电感(H)
#define MOTOR_PSI_F 0.0035f // 永磁体磁链(Wb)

#define PHASE_SWAP 1            // 1：启用相序翻转
#define ENCODER_COUNT_SWAP 0    // 1：启用编码器计数翻转
#define FOC_DECOUPLING_ENABLE 1 // 1:启用前馈解耦 0:关闭前馈解耦

#endif /* __MOTOR_CONFIG_H__ */