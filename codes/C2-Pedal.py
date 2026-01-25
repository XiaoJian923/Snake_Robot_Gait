'''
说明：MuJoCo仿真实现C2-足波步态，从形式来看，本步态与S-Pedal步态如出一辙
参考文献：2023 面向灾后环境的蛇形机器人重构设计及运动控制研究
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923
时间：20260125
'''
import math
import time
import mujoco
import bisect
import numpy as np
import mujoco.viewer
from scipy import integrate


l = 0.064
Rp = 0.11

alpha_p = math.radians(60)      # 弧度制

Psi_array = [0, -math.pi / 2, math.pi / 2, 0, math.pi / 2, -math.pi / 2]

# 计算6个简单曲线段长度的累加，用于定位弧长s处于第几个简单曲线段
sum = 0
length_array = [(math.pi - alpha_p) / 2 * Rp, math.pi * Rp, (math.pi - alpha_p) / 2 * Rp, (math.pi - alpha_p) / 2 * Rp, math.pi * Rp, (math.pi - alpha_p) / 2 * Rp]
accumulate_array = []
for i in length_array:
    sum += i
    accumulate_array.append(sum)


# 计算在弧长s下，当前正处于第几个简单曲线段
def calc_n(s):
    n = bisect.bisect_left(accumulate_array, s % sum)
    return n + 1


# 计算扭转角
def Psi(s):
    temp = 0
    n = calc_n(s)
    for i in range(n):
        temp += Psi_array[i]
    return temp


# 判断当前弧长s处于第几个简单曲线段，并返回对应曲率
def K(s):
    return 1 / Rp


# 返回当前弧长s所对应的挠率
def T(s):
    return 0


# 俯仰关节对应曲率
def Kp(s):
    if s < 0:
        return 0
    return -1 * K(s) * math.sin(Psi(s))


# 偏航关节对应曲率
def Ky(s):
    if s < 0:
        return 0
    return K(s) * math.cos(Psi(s))


s_h = 0             # 距离头部关节前段的位置，改变s_h的值（如递增操作），即可实现shift control
delta_s = l * 0.4   # 实现shift control时每次递增的增量
N = 40              # 蛇形机器人的关节数量

time_sleep = 0.05   # 每epoch睡眠时间
epochs = 800        # 循环次数 800
steps = 200         # step次数


m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o40_C2-Pedal.xml')
d = mujoco.MjData(m)
data_ctrl = np.zeros(N)


with mujoco.viewer.launch_passive(m, d) as viewer:
    for i in range(epochs):
        for j in range(1, N + 1): # 计算关节角度
            if j % 2 == 1:
                angle_rad = integrate.quad(Kp, s_h - (j + 1) * l, s_h - (j - 1) * l, limit=200)
            else:
                angle_rad = integrate.quad(Ky, s_h - (j + 1) * l, s_h - (j - 1) * l, limit=200)
            data_ctrl[j - 1] = angle_rad[0]
        with viewer.lock():
            d.ctrl = data_ctrl
            for i in range(steps):
                mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(time_sleep)
        s_h += delta_s

