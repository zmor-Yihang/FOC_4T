---
name: angle-lag-compensation
overview: 针对编码器角度获取返回缓存值导致的控制滞后，制定一套基于当前工程结构的延迟分析、角度前推补偿与采样时序优化方案，优先采用软件侧低风险改法。
todos:
  - id: refactor-encoder-observer
    content: 重构 User/bsp/encoder.c/.h，分离测量角、估计角与控制前推角
    status: completed
  - id: implement-delay-compensation
    content: 在 encoder_update 中实现真实校正与可配置延迟补偿
    status: completed
    dependencies:
      - refactor-encoder-observer
  - id: update-foc-angle-callers
    content: 修改 current_closed.c、speed_closed.c、foc.c，统一角度调用语义
    status: completed
    dependencies:
      - implement-delay-compensation
  - id: tune-and-verify
    content: 标定补偿时间并验证低速、高速稳定性与安全回退
    status: completed
    dependencies:
      - update-foc-angle-callers
---

## 用户需求

- 当前角度获取依赖上一拍已完成的缓存结果，导致控制链路存在等效一周期滞后。
- 需要在现有工程范围内降低角度反馈延迟，让电流环、速度环使用更接近当前时刻的电角度。
- 优先通过软件方案解决，不默认更换传感器、接口或整体控制架构。
- 需要同时保留调试能力，能区分原始测量角、估计角和控制实际使用的角度。

## 产品概述

- 优化角度观测链路，把“编码器测量值”“PLL估计值”“控制前推角”职责拆开，避免缓存值直接形成控制滞后。
- 无界面变化，效果主要体现在调试波形中角度与电流对齐更好，高速下相位滞后、抖动和闭环压力减小。

## 核心功能

- 分离原始测量角、观测器估计角、控制用补偿角
- 对编码器总延迟做可配置的角度前推补偿
- 在无新样本时仅做预测传播，不把预测值当作真实测量参与校正
- 保留补偿开关与调试输出，便于标定和回退

## 技术栈选择

- 平台：STM32G431 现有裸机/HAL 工程
- 语言：C
- 控制链路：`TIM2/TIM3` 中心对齐 PWM，`ADC` 注入组回调执行 FOC
- 角度链路：`I2C3` 异步读取 `AS5600`，`encoder_update()` 在控制回调中更新观测器
- 已确认约束：
- `User/bsp/encoder.c` 当前在 `I2C_READ_STATE_DONE` 时消费 `i2c_rx_buf`，随后再启动下一次异步读，天然形成异步流水线延迟
- `User/motor/current_closed.c`、`User/motor/speed_closed.c` 控制实际使用的是 `encoder_get_angle()` 返回的 PLL 估计角
- `User/bsp/adc.c` 通过 `adc_injected_callback_flag` 对用户回调做分频执行，`ENCODER_SPEED_SAMPLE_TIME` 需要与实际控制节拍保持一致

## 实现方式

### 总体策略

保留现有非阻塞 I2C 流水线，不改成阻塞式读编码器；重点重构编码器观测逻辑，把“新测量到达时的校正”和“每个控制周期的相位预测”解耦，并在导出控制角时增加可配置前推补偿。

### 高层工作机制

1. 每个控制周期先按当前电角速度传播 PLL 相位  
2. 仅当 `I2C_READ_STATE_DONE` 时，才把最新原始角度作为测量值参与 PLL 校正  
3. `I2C_READ_STATE_BUSY` 时不再把预测角伪装成测量角，避免把自预测误差反灌进 PLL  
4. 控制使用角度改为 `估计角 + 电角速度 × 总延迟补偿时间`，从而抵消缓存/总链路滞后

### 关键技术决策

- **保留异步读取**：阻塞式 I2C 会直接占用 FOC 回调时间，增大中断抖动和实时性风险；现有结构更适合做观测补偿而不是同步阻塞。
- **补偿放在控制输出侧**：PLL 仍用真实测量做校正，前推只作用于控制角，能减少相位滞后且不容易破坏观测器稳定性。
- **接口语义拆分**：明确提供“测量角/估计角/控制角”三类输出，避免当前 `current_elec_angle` 同时承担测量值和预测值造成语义混乱。
- **参数化延迟**：将控制前推时间做成宏配置，先覆盖一拍缓存延迟，再视调试结果补偿 I2C 事务和传感器内部延迟。

### 性能与可靠性

- 单周期新增计算量为常数级 `O(1)`，仅增加少量浮点加减乘和包角处理。
- 不引入动态内存，不增加阻塞等待，不改变主中断拓扑。
- 主要瓶颈仍是传感器本体与 I2C 链路延迟；若软件补偿后高速段仍不足，再评估更高实时性的编码器接口作为后续方案。
- 需要重点防止：
- 速度估计过大时前推角过冲
- `ENCODER_SPEED_SAMPLE_TIME` 与真实控制节拍不一致
- 对齐流程误用补偿角导致零偏标定失真

## 实施说明

- 复用现有 `angle_wrap_0_2pi`、`angle_wrap_pm_pi` 工具函数，避免引入新数学路径。
- 中断内不增加打印或阻塞操作；调试仍通过主循环打印已有缓存变量完成。
- 保持最小改动范围，优先集中在编码器层与角度调用点，不做无关 ADC/TIM 重构。
- 为补偿逻辑提供安全开关和默认参数，便于快速回退到当前行为。

## 架构设计

现有修改后的数据流建议为：

- `I2C 最新完成样本` → `原始电角度 measured_angle`
- `PLL 预测/校正` → `估计角 observer_angle + 估计速度`
- `延迟补偿` → `控制角 control_angle`
- `current_closed/speed_closed` 使用 `control_angle`
- `foc_alignment` 与调试观察继续使用 `measured_angle` 或 `observer_angle`

## 目录结构

### Directory Structure Summary

本次实现聚焦编码器观测层与 FOC 角度调用点，目标是在不改变现有控制框架的前提下消除等效一拍滞后。

```text
c:\Users\30495\Desktop\FOC_4T\
└── User/
    ├── bsp/
    │   ├── encoder.h          # [MODIFY] 编码器接口与参数定义。新增/整理控制前推补偿参数、角度输出接口声明，明确测量角、估计角、控制角的语义边界。
    │   └── encoder.c          # [MODIFY] 编码器核心观测逻辑。重构 encoder_update()：无新样本时只做预测传播；有新样本时执行 PLL 校正；生成可配置的控制前推角并保持包角一致。
    ├── motor/
    │   ├── current_closed.c   # [MODIFY] 电流环回调角度来源切换。控制使用补偿后的控制角，调试保留测量角/估计角对比，避免混用旧接口。
    │   └── speed_closed.c     # [MODIFY] 速度环回调角度来源切换。与电流环保持一致，并继续输出调试所需的角度和速度观测结果。
    └── foc/
        └── foc.c              # [MODIFY] 对齐流程使用明确的测量角接口，避免把控制前推角引入零偏扫描与标定流程。
```