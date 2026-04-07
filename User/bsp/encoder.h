#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32g4xx_hal.h"
#include "motor_config.h"
#include "i2c.h"

// AS5600寄存器地址定义
#define AS5600_I2C_ADDR (0x36 << 1) // AS5600的7位I2C地址是0x36，HAL中左移1位使用

#define AS5600_REG_ZMCO 0x00   // OTP烧录计数寄存器，记录ZPOS/MPOS永久烧录次数
#define AS5600_REG_ZPOS_H 0x01 // 零位起始角高字节，设置输出起点位置[11:8]
#define AS5600_REG_ZPOS_L 0x02 // 零位起始角低字节，设置输出起点位置[7:0]
#define AS5600_REG_MPOS_H 0x03 // 最大位置高字节，设置输出终点位置[11:8]
#define AS5600_REG_MPOS_L 0x04 // 最大位置低字节，设置输出终点位置[7:0]
#define AS5600_REG_MANG_H 0x05 // 最大角度范围高字节，设置有效测量角度范围[11:8]
#define AS5600_REG_MANG_L 0x06 // 最大角度范围低字节，设置有效测量角度范围[7:0]
#define AS5600_REG_CONF_H 0x07 // 配置寄存器高字节，包含看门狗、快速滤波阈值、慢滤波设置
#define AS5600_REG_CONF_L 0x08 // 配置寄存器低字节，包含PWM频率、输出模式、磁滞、功耗模式

#define AS5600_REG_STATUS 0x0B      // 状态寄存器，指示磁铁是否检测到、磁场过强或过弱
#define AS5600_REG_RAW_ANGLE_H 0x0C // 原始角度高字节，未经零位/量程缩放处理的12位原始角度[11:8]
#define AS5600_REG_RAW_ANGLE_L 0x0D // 原始角度低字节，未经零位/量程缩放处理的12位原始角度[7:0]
#define AS5600_REG_ANGLE_H 0x0E     // 输出角度高字节，经过零位/量程配置后的12位角度值[11:8]
#define AS5600_REG_ANGLE_L 0x0F     // 输出角度低字节，经过零位/量程配置后的12位角度值[7:0]

#define AS5600_REG_AGC 0x1A   // 自动增益控制寄存器，用于反映当前磁场强弱补偿值
#define AS5600_REG_MAG_H 0x1B // 磁场幅值高字节，内部CORDIC计算得到的磁场强度[11:8]
#define AS5600_REG_MAG_L 0x1C // 磁场幅值低字节，内部CORDIC计算得到的磁场强度[7:0]

#define AS5600_REG_BURN 0xFF // OTP烧录命令寄存器，写0x80烧录角度，写0x40烧录配置

#define AS5600_STATUS_MH (1 << 3) // STATUS位3：磁场过强
#define AS5600_STATUS_ML (1 << 4) // STATUS位4：磁场过弱
#define AS5600_STATUS_MD (1 << 5) // STATUS位5：检测到磁铁

#define AS5600_BURN_ANGLE 0x80   // 烧录ZPOS/MPOS命令
#define AS5600_BURN_SETTING 0x40 // 烧录MANG/CONF命令

// 速度计算参数
#define ENCODER_CPR 4096
#define ENCODER_TWO_PI 6.28318530718f

// 编码器方向修正系数：当机械安装方向或电角度定义方向与FOC控制所需正方向相反时设为-1，
// 相同则设为1；该系数会乘到编码器读数上，用于统一修正角度与转速符号方向
#define ENCODER_DIRECTION (-1)

// 速度计算分频系数：`encoder_update_speed()` 必须在固定周期中断里按 `ENCODER_SPEED_SAMPLE_TIME` 对应的基础周期稳定调用一次，
#define SPEED_CAL_DIV 20

// 编码器速度基础采样周期，单位秒：它表示 `encoder_update_speed()` 两次相邻调用之间的理论时间间隔；
// ENCODER_SPEED_SAMPLE_TIME = 电流环周期 * SPEED_CAL_DIV
#define ENCODER_SPEED_SAMPLE_TIME 0.0001f

// 速度一阶低通滤波系数
// y(k)=α*x(k)+(1-α)*y(k-1)
#define ENCODER_SPEED_FILTER_ALPHA 0.1f

void encoder_init(void);

void encoder_update_speed(void);
float encoder_get_angle_rad(void);
float encoder_get_speed_rpm(void);

#endif /* encoder.h */