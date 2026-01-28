'''
说明：MuJoCo仿真实现螺旋步态，由于没有搭建管道，因此仿真效果较差;论文仅在水平管上做了实验
参考文献：2025 Novel Gaits for Snake Robot Navigation in Complex External Pipe Networks
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923
时间：20260128
'''
import time
import math
import mujoco
import numpy as np
import mujoco.viewer

N = 20              # 关节数
A = 1.23
omega_t = 4.5
omega_s = 0.11
l_M = 70            # 连杆长度
p = l_M / (((A / (2 * math.sin(omega_s))) ** 2 + 1) * omega_s)
r = A / (2 * math.sin(omega_s)) * p

k_r = 10
k_p = 0.75
m = 30
# Delta_T_s = 120
p_omega_s = k_p * p * l_M / ((k_r * r) ** 2 + (k_p * p) ** 2)
p_A = 2 * k_r * r / (k_p * p) * math.sin(p_omega_s)
velocity = 2

def Function_S(n, n_s):
    a = 1 / (1 + math.exp(-m * (n - (n_s - 0.5))))
    b = 1 / (1 + math.exp(-m * ((n_s + 0.5) - n)))
    return a + b - 1


t = 0
delta_t = 0.04
n_s = N - 1

time_sleep = 0.02
epochs = 200  # 循环次数
steps = 10  # step次数

M = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o20_Spiraling.xml')
d = mujoco.MjData(M)
data_ctrl = np.zeros(N)

with mujoco.viewer.launch_passive(M, d) as viewer:
    for i in range(epochs):
        for n in range(N):
            # 以下计算分为两部分
            if n % 2 == 0:
                angle_rad = A * np.sin(omega_t * t + omega_s * n) * (1 - Function_S(n, n_s)) + p_A * np.sin(omega_t * t + p_omega_s * n) * Function_S(n, n_s)
            else:
                angle_rad = A * np.sin(omega_t * t + omega_s * n + np.pi / 2) * (1 - Function_S(n, n_s)) + p_A * np.sin(omega_t * t + p_omega_s * n + np.pi / 2) * Function_S(n, n_s)
            if n_s < 0:
                n_s = N - 1
            else:
                n_s -= velocity * delta_t
            data_ctrl[n] = angle_rad
            with viewer.lock():
                d.ctrl = data_ctrl
                for i in range(steps):
                    mujoco.mj_step(M, d)
                viewer.sync()
                time.sleep(time_sleep)
        t += delta_t

