# FOC_2804 — 基于 STM32G431 的磁场定向控制（FOC）电机驱动

本项目是一款基于 **STM32G431RB** 微控制器的无刷电机（BLDC/PMSM）FOC 驱动方案，支持 **有感电流闭环、有感速度闭环、无感速度闭环** 三种运行模式。硬件上使用 **AS5600 磁编码器** 进行转子位置检测，电流采样通过片内 ADC 实现双电阻采样。

---

## 主要特性

- **电流环频率**：10 kHz
- **速度环频率**：1 kHz（电流环的 1/10 分频）
- **控制模式**：
  - 有感电流闭环（`currentClosed`）
  - 有感速度闭环（`speedClosed`）
  - 无感速度闭环（`fluxObserverClosed`，基于磁链观测器）
- **调制方式**：零序注入SVPWM
- **解耦控制**：支持 D/Q 轴前馈解耦
- **弱磁控制**：支持弱磁扩展转速（待完善）
- **角度插值**：针对 AS5600 I2C 通信速率瓶颈，设计二阶锁相环实时解算电角速度
- **调试输出**：支持 VOFA+ 波形调试（通过 USART）

---

## 项目结构

```
FOC_2804/
├── User/
│   ├── app/              # 应用层：main 入口、用户配置
│   ├── bsp/              # 板级支持包：ADC、GPIO、TIM、USART、I2C、时钟
│   ├── sensor/           # 传感器驱动：AS5600 编码器、电流采样
│   ├── alg/              # 基础算法：Clarke/Park 变换、SVPWM、PID
│   ├── foc/              # FOC 核心：控制对象、环路调度、零点校准、栅极驱动
│   ├── motor/            # 应用级闭环：电流闭环、速度闭环、无感闭环
│   ├── adv_alg/          # 高级算法：磁链观测器、弱磁控制
│   └── utils/            # 工具库：快速三角函数、角度工具、斜坡、FIFO、打印
├── Drivers/              # STM32 HAL 库及 CMSIS
├── docs/                 # 调试记录与开发文档
├── 芯片手册/             # 芯片数据手册与参考手册
└── STM32G431RBTX_FLASH.ld  # 链接器脚本
```
---

## 目录说明

| 目录 | 说明 |
|------|------|
| `User/app` | `main.c` 程序入口，`user_config.h` 全局配置 |
| `User/bsp` | 底层外设初始化：时钟、GPIO、TIM（PWM）、ADC、USART、I2C |
| `User/sensor` | AS5600 编码器读取、电流采样及转换 |
| `User/alg` | Clarke/Park 坐标变换、SVPWM、PID 控制器 |
| `User/foc` | FOC 核心结构体、电流/速度环调度、零点校准、栅极驱动使能 |
| `User/motor` | 三种应用级闭环控制的封装（电流/速度/无感） |
| `User/adv_alg` | 磁链观测器（用于无感控制）、弱磁控制算法 |
| `User/utils` | 快速 sin/cos、角度归一化、斜坡函数、VOFA 打印、FIFO |

---

## 快速开始

### 1. 环境准备

- **IDE**：VS Code + EIDE 插件 + Cortex-Debug 插件
- **编译器**：GCC
- **调试器**：OpenOCD
- **串口助手**：VOFA+

### 2. 编译与烧录

1. 使用 VS Code 打开 `FOC_2804.code-workspace`
2. 在 EIDE 中选择对应的编译器工具链
3. 点击 **Build** 编译项目
4. 点击 **Download** 烧录固件

### 3. 选择运行模式

在 `User/app/main.c` 中修改初始化函数，三选一：

```c
// 电流闭环（仅控制 Id/Iq）
currentClosed_init(0.0f, 0.5f);

// 速度闭环（有感，编码器提供速度）
speedClosed_init(3000);

// 无感速度闭环（磁链观测器）
fluxObseverClosed_init(30);
```

同时，在 `while(1)` 中选择对应的调试输出：

```c
currentClosedDebug_print_info();      // 电流闭环调试
speedClosedDebug_print_info();        // 速度闭环调试
fluxObseverClosedDebug_print_info();  // 无感闭环调试
```

---

## 参数配置

电机与控制参数集中在 `User/app/user_config.h`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `MOTOR_POLE_PAIRS` | 7 | 电机极对数 |
| `MOTOR_RS_Ω` | 2.5 Ω | 定子电阻 |
| `MOTOR_LD_H` / `MOTOR_LQ_H` | 0.86 mH | D/Q 轴电感 |
| `MOTOR_PSI_F` | 0.0035 Wb | 永磁体磁链 |
| `FOC_CURRENT_LOOP_FREQ_HZ` | 10000 | 电流环频率 |
| `FOC_SPEED_LOOP_DIVIDER` | 10 | 速度环分频系数 |
| `FOC_DECOUPLING_ENABLE` | 1 | 前馈解耦开关 |

---

## 开发计划

- **通信与控制链路延迟补偿**：针对 I2C 读取与 PWM 更新固有延迟，基于实时转速的动态相位超前补偿策略，精准抵消转子位置滞后，保障 FOC 换相定向精度与环路稳定性。

- **齿槽转矩离散映射前馈补偿**：通过离线匀速标定构建电角度-齿槽转矩映射表，提取周期性脉动特征；在 FOC q 轴电流参考值中实时注入查表前馈量，抑制低速转矩脉动，提升运行平稳性。

- **龙伯格观测器扰动前馈抑制**：建立含负载转矩的电机状态空间模型，设计全阶龙伯格观测器估计外部扰动力矩，补偿负载突变与外部扰动，增强低速工况下的系统鲁棒性。

