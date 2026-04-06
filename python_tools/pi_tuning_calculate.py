import math

# =========================================================
# 电机参数（SI单位）
# =========================================================
R     = 2.55          # 相电阻 [Ohm]
L     = 0.86e-3       # 相电感 [H]
Ld    = 0.86e-3       # d轴电感 [H]
psi_f = 0.0035        # 永磁磁链 [Wb]
P     = 7             # 极对数
J     = 3.7e-6        # 转动惯量 [kg*m^2]

# =========================================================
# 控制周期
# =========================================================
Ts_current = 1.0 / 20000.0   # 电流环执行周期 [s]，20kHz

# 如果速度环降频执行，比如每10个电流环执行1次：
speed_loop_div = 10
Ts_speed = Ts_current * speed_loop_div   # 速度环执行周期 [s]

# =========================================================
# 设计目标
# =========================================================
current_bw_hz = 500.0       # 电流环带宽 [Hz]

# 速度环带宽 = 电流环带宽 / speed_ratio
# 可选：10 ~ 15
speed_ratio = 5

# 也可以直接指定 delta（与 speed_ratio 等价）
# delta = speed_ratio
delta = speed_ratio

# =========================================================
# 单位换算
# =========================================================
RPM_TO_RAD_PER_SEC = 2.0 * math.pi / 60.0
RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * math.pi)

# =========================================================
# 电流环整定（TI InstaSPIN 文档方法）
# =========================================================
# Kp = wc * L
# Ki = R / L
# 这里 Ki 为连续域积分系数 [1/s]
# Ki*Ts 为数字实现常用参数
wc_current = 2.0 * math.pi * current_bw_hz   # [rad/s]

current_kp = wc_current * L                  # [V/A]（若输出是电压，输入是电流）
current_ki = R / L                           # [1/s]
current_ki_ts = current_ki * Ts_current      # 离散实现常用

# =========================================================
# 速度环整定（TI InstaSPIN / 阻尼因子法）
# =========================================================
# 电机力矩常数：
# T = Kt * iq
# 对于PMSM，Kt = 3/2 * P * psi_f
Kt = 1.5 * P * psi_f                         # [N*m/A]

# TI文档里的 K
# A = 3*P*psi_f / (4*J)
# 这个 A 的单位是 [rad/s^2 per A]
A = 3.0 * P * psi_f / (4.0 * J)

# 速度环带宽近似
speed_bw_hz = current_bw_hz / delta
wc_speed = 2.0 * math.pi * speed_bw_hz       # 仅用于显示，不直接参与TI公式

# TI公式：
# speed_ki = current_kp / (Ld * delta^2)
# speed_kp = delta * speed_ki / A
#
# 这里 speed_ki 是连续域积分系数 [1/s]
speed_ki_rad = current_kp / (Ld * delta * delta)    # [1/s]
speed_kp_rad = delta * speed_ki_rad / A             # [A / (rad/s)]

speed_ki_ts_rad = speed_ki_rad * Ts_speed           # 速度环实际执行周期下 Ki*Ts

# =========================================================
# 如果速度反馈单位用 RPM，则参数要换算
# =========================================================
# 因为：
# e_rad = e_rpm * (2*pi/60)
# 为保持输出不变：
# Kp_rpm = Kp_rad * (2*pi/60)
# Ki_rpm = Ki_rad * (2*pi/60)
speed_kp_rpm = speed_kp_rad * RPM_TO_RAD_PER_SEC
speed_ki_rpm = speed_ki_rad * RPM_TO_RAD_PER_SEC
speed_ki_ts_rpm = speed_ki_rpm * Ts_speed

# =========================================================
# 输出
# =========================================================
print("=" * 60)
print("电机参数")
print("=" * 60)
print(f"R      = {R:.6f} Ohm")
print(f"L      = {L:.6e} H")
print(f"Ld     = {Ld:.6e} H")
print(f"psi_f  = {psi_f:.6e} Wb")
print(f"P      = {P:d} pole-pairs")
print(f"J      = {J:.6e} kg*m^2")
print(f"Kt     = {Kt:.6f} N*m/A")
print()

print("=" * 60)
print("控制周期")
print("=" * 60)
print(f"Ts_current = {Ts_current:.8f} s  ({1.0/Ts_current:.1f} Hz)")
print(f"Ts_speed   = {Ts_speed:.8f} s  ({1.0/Ts_speed:.1f} Hz)")
print()

print("=" * 60)
print("电流环 PI")
print("=" * 60)
print(f"目标带宽 f_ci = {current_bw_hz:.2f} Hz")
print(f"角频率   w_ci = {wc_current:.2f} rad/s")
print()
print(f"Kp        = {current_kp:.6f}")
print(f"Ki        = {current_ki:.6f}  [1/s]")
print(f"Ki*Ts     = {current_ki_ts:.6f}  (Ts = Ts_current)")
print()

print("=" * 60)
print("速度环 PI（TI / 阻尼因子法）")
print("=" * 60)
print(f"delta          = {delta:.2f}")
print(f"速度环带宽约   = {speed_bw_hz:.2f} Hz")
print(f"速度环角频率约 = {wc_speed:.2f} rad/s")
print()

print("[若速度输入单位 = rad/s]")
print(f"Kp        = {speed_kp_rad:.6f}  [A / (rad/s)]")
print(f"Ki        = {speed_ki_rad:.6f}  [1/s]")
print(f"Ki*Ts     = {speed_ki_ts_rad:.6f}  (Ts = Ts_speed)")
print()

print("[若速度输入单位 = RPM]")
print(f"Kp        = {speed_kp_rpm:.6f}  [A / RPM]")
print(f"Ki        = {speed_ki_rpm:.6f}  [RPM制下连续积分系数, 1/s]")
print(f"Ki*Ts     = {speed_ki_ts_rpm:.6f}  (Ts = Ts_speed)")
print()

print("=" * 60)
print("建议用于你的 pid_init()")
print("=" * 60)
print("电流环（每20kHz执行一次）:")
print(f"  pid_init(&pid_id, {current_kp:.4f}f, {current_ki_ts:.6f}f, -Vmax, Vmax);")
print(f"  pid_init(&pid_iq, {current_kp:.4f}f, {current_ki_ts:.6f}f, -Vmax, Vmax);")
print()

print("速度环（如果输入是 RPM，且每 speed_loop_div 个电流环执行一次）:")
print(f"  pid_init(&pid_speed, {speed_kp_rpm:.6f}f, {speed_ki_ts_rpm:.6f}f, -Imax, Imax);")
print()
