# FOC 控制系统解耦架构设计说明

在现代电机驱动软件设计中，将硬件相关代码、纯数学控制算法以及顶层的业务逻辑完全解耦，是保证代码具有**高可移植性（跨MCU/跨RTOS）**、**高可复用性（支持多电机实例）**与**高可测试性（支持纯软件SIL仿真）**的必由之路。

标准的解耦方向就是将业务逻辑、数学算法与硬件驱动三者完全分离，层与层之间**禁止直接越级调用或直接引用HAL库**，只通过标准化的数据结构（结构体）传递输入与输出。

## 1. 架构分层概述

系统应当严格划分为以下三个隔离的层次：

1. **硬件抽象层 (Hardware/Driver Layer)**：负责直接操作单片机寄存器、读取 ADC（相电流）、读取各种传感器（SPI/编码器）并控制定时器（下发PWM占空比）。
2. **纯算法核心层 (Algorithm/Math Layer)**：只做纯数学运算，没有任何对硬件底层函数的直接调用。比如 Clark/Park 变换、SVPWM 计算、PID 计算、观测器迭代。
3. **业务逻辑与模式层 (Application/Control Layer)**：负责 FOC 状态机调度、保护机制处理、不同的控制模式（电流、速度、位置）、轨迹规划以及低频的外环（速度/位置环）计算。

---

## 2. 核心标准化结构体 (中间传输载体)

这些结构体作为层与层之间的“接口协议”，定义在通用头文件中，比如 `foc_types.h`。各层仅仅是对这些数据的消费和生产。

```c
/* ================= 标准化数据类型定义 ================= */

// 三相电流/电压/占空比
typedef union {
    struct { float a; float b; float c; };
    float value[3];
} abc_t;

// 静止坐标系 (Alpha-Beta)
typedef struct {
    float alpha;
    float beta;
} alphabeta_t;

// 旋转坐标系 (D-Q)
typedef struct {
    float d;
    float q;
} dq_t;

// 标准化传感器反馈数据 (由硬件抽象层产出，喂给算法和控制层)
typedef struct {
    float elec_angle_rad;   /* 当前电角度 (rad) */
    float mech_angle_rad;   /* 机械角度 (rad) */
    float elec_speed_rad_s; /* 电角速度 (rad/s) */
    float mech_speed_rpm;   /* 机械角速度 (rpm) */
} sensor_feedback_t;

// FOC 电流环目标与状态指令 (由业务层产出，喂给底层 FOC 电流环)
typedef struct {
    float target_id;
    float target_iq;
    uint8_t enable_pwm;     /* 是否使能最终的占空比输出 */
} foc_cmd_t;
```

---

## 3. 各层级设计与接口定义

### 3.1 纯算法核心层 (FOC Math & Inner Loop)
该层是纯 C 代码，完全不知道自己运行在 STM32 还是 ESP32 上，也不知道电流是 ADC 读来的，只负责“算”。

```c
/* foc_core.h */

// 解耦后的 FOC 核心对象
typedef struct {
    pid_controller_t pid_id;
    pid_controller_t pid_iq;
    float dc_bus_voltage;   /* 母线电压 */
} foc_core_t;

/**
 * @brief FOC 纯数学单步核心计算 (等效于电流环+SVPWM)
 * @param foc         FOC 核心状态结构体
 * @param i_abc       采样的三相电流 (输入)
 * @param sensor_fb   当前传感器角度与速度快照 (输入)
 * @param cmd         目标 DQ 轴电流 (输入)
 * @return abc_t      计算完成的三相 PWM 占空比 (0.0f ~ 1.0f) (输出)
 */
abc_t foc_core_step(foc_core_t *foc, abc_t i_abc, sensor_feedback_t sensor_fb, foc_cmd_t cmd);
```

### 3.2 业务逻辑层 (Application Control Loop)
这一层负责处理用户的不同控制模式（速度模式、位置模式），管理外环 PI 控制器。它不需要自己去调用 PWM 硬件接口，而是把外环的计算结果（目标 Qd 电流）打包好等待被执行。

```c
/* motor_app.h */

typedef struct {
    motor_mode_t mode;
    pid_controller_t pid_speed;
    pid_controller_t pid_position;
    // ... 其他轨迹规划参数和保护状态
} motor_app_t;

/**
 * @brief 业务控制层调度 (按预定的时间周期执行，例如 1kHz)
 * @param app         业务逻辑结构体
 * @param sensor_fb   当前传感器反馈 (输入)
 * @param target      用户的最终目标(比如目标位置/速度) (输入)
 * @return foc_cmd_t  外环计算后产生的目标 Iq, Id 电流指令 (输出)
 */
foc_cmd_t motor_app_step(motor_app_t *app, sensor_feedback_t sensor_fb, float target);
```

### 3.3 硬件抽象层适配器 (Hardware Adapter)
这是实际与单片机硬件打交道的地方。例如在 STM32 的 ADC 注入转换完成中断中，我们将这三者串联起来：获取底层数据，送入解耦算法，拿着算法结果去设置底层外设。

```c
/* hardware_adapter.c (运行在 STM32 特定的平台下) */

// 硬件中断回调例程 (比如 10kHz ADC 注入中断)
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    abc_t i_abc;
    sensor_feedback_t sensor_fb;
    abc_t final_duty;
    foc_cmd_t current_cmd;

    // 1. [读底层硬件]: 从 ADC 寄存器读取原始电流值并标定为安培
    i_abc = hw_adc_get_phase_currents();

    // 2. [读底层硬件]: 从 SPI 或定时器等读取编码器角度与速度
    sensor_fb = hw_encoder_get_feedback();

    // 3. [业务逻辑(可选或降频跑)]: 获取最新生成的目标 ID/IQ
    current_cmd = get_latest_foc_cmd_from_app();

    // 4. [调纯算法]: 将结构体丢给“无关硬件的 FOC 核心算子”
    final_duty = foc_core_step(&motor_foc_core, i_abc, sensor_fb, current_cmd);

    // 5. [写底层硬件]: 将纯算法计算出的 0~1 的占空比更新给 Timer 比较寄存器
    if (current_cmd.enable_pwm) {
        hw_pwm_set_duty(final_duty);
    } else {
        hw_pwm_disable();
    }
}
```

---

## 4. 架构解耦的好处演示：多传感器融合或切换

采用这种标准结构体传递的解耦架构后，你会发现如果需要切换有感（编码器）和无感（观测器）变得非常符合自然直觉：无论是编码器计算，还是无感观测器计算，最终产出的都是一个**标准的 `sensor_feedback_t` 对象**。

```c
sensor_feedback_t sensor_fb;

if (use_sensorless) {
    // 喂电流进去，吐出包含虚拟角度/速度的统一数据包
    sensor_fb = observer_get_feedback(i_abc);
} else {
    // 读 SPI 磁编码器，吐出包含实际角度/速度的统一数据包
    sensor_fb = hw_encoder_get_feedback();
}

// FOC 永远只接受统一格式的数据包，核心算法不动如山
abc_t duty = foc_core_step(&motor_foc, i_abc, sensor_fb, cmd);
```

通过这种层级约束设计：
- 如果我们要把这个代码迁移到 ESP32，只需要重写 `hw_adc_get_phase_currents()` 和 `hw_pwm_set_duty()` 这两个获取/下发硬件接口即可。
- 如果我们要做 Simulink 或者 PC 上的纯软件环路测试 (SIL)，只需提供模拟的假电流和假角度数据传递进 `foc_core_step` 函数，即可验证算法。