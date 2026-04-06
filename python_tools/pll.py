def calc_observer_pi(max_electrical_freq, observer_sampling_freq):
    p_gain = 532 * max_electrical_freq / observer_sampling_freq
    i_gain = 1506742 * max_electrical_freq / (observer_sampling_freq ** 2)
    return p_gain, i_gain



max_electrical_freq = 1000.0      # 最大电角频率
observer_sampling_freq = 20000.0  # 观测器采样频率

p_gain, i_gain = calc_observer_pi(max_electrical_freq, observer_sampling_freq)

print("P gain =", p_gain)
print("I gain =", i_gain)