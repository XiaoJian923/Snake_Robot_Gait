'''
说明：MuJoCo仿真实现履带步态的衍生步态，H-Crawler
参考文献：2023 Extension and Experimental Demonstration of Gait Transition Network for a Snake Robot
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923
时间：20260123
'''
import math
import time
import mujoco
import bisect
import numpy as np
import mujoco.viewer
from scipy import integrate


l = 70          # 关节长度
l_s = 300       # 直线长度
R_H = 85        # 螺旋线半径
P_H = 160        # 螺旋线螺距

k_max = 1.5708 / (2 * l)

slope = P_H / (2 * math.pi)
denominator_helix = R_H ** 2 + slope ** 2

k_helix = R_H / denominator_helix       # 螺旋线曲率
tau_helix = slope / denominator_helix   # 螺旋线挠率
l_helix = 2 * math.pi * math.sqrt(denominator_helix)  # 螺旋线长度
twist_helix = l_helix * tau_helix  # 对螺旋线的挠率进行积分，螺旋线整体挠率

print("螺旋曲线曲率k_helix：", k_helix)
print("当前连杆长度下最大曲率k_max：", k_max)
if k_helix > k_max:
    print("螺旋曲线曲率过大，应当重新选择构型参数！")
    exit(0)

# 计算2个简单曲线段长度的累加，用于定位弧长s处于第几个简单曲线段 直线为特殊的曲线
sum = 0
length_array = [l_s, l_helix, l_s, l_helix]
accumulate_array = []
for i in length_array:
    sum += i
    accumulate_array.append(sum)


# 计算在弧长s下，当前正处于第几个简单曲线段
def calc_n(s):
    n = bisect.bisect_left(accumulate_array, s % sum)
    return n + 1


# 计算扭转角 (8)
def Psi(s):
    n = calc_n(s)
    if n == 1:
        return 0
    elif n == 2:
        return (s % sum - length_array[0]) * tau_helix
    elif n == 3:
        return twist_helix
    else:
        return twist_helix - (s % sum - accumulate_array[2]) * tau_helix


# 判断当前弧长s处于第几个简单曲线段，并返回对应曲率
def K(s):
    n = calc_n(s)
    if n in [1, 3]:# 直线段
        return 0
    else:  # 螺旋线段
        return k_helix


# 返回当前弧长s所对应的挠率
def T(s):
    return 0


# 俯仰关节对应曲率 (1)
def Kp(s):
    if s < 0:
        return 0
    return -1 * K(s) * math.sin(Psi(s))


# 偏航关节对应曲率 (2)
def Ky(s):
    if s < 0:
        return 0
    return K(s) * math.cos(Psi(s))


s_h = 0         # 距离头部关节前段的位置，改变s_h的值（如递增操作），即可实现shift control
delta_s = l * 0.2    # 实现shift control时每次递增的增量
N = 40          # 蛇形机器人的关节数量

time_sleep = 0.05   # 每epoch睡眠时间
epochs = 800        # 循环次数
steps = 50         # step次数

m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o40_H-Crawler.xml')
d = mujoco.MjData(m)
data_ctrl = np.zeros(N)

with mujoco.viewer.launch_passive(m, d) as viewer:
    for i in range(epochs):
        for j in range(1, N + 1): # 计算关节角度 (4)
            if j % 2 == 1:
                angle_rad = integrate.quad(Kp, s_h - (j + 1) * l, s_h - (j - 1) * l)
            else:
                angle_rad = integrate.quad(Ky, s_h - (j + 1) * l, s_h - (j - 1) * l)
            data_ctrl[j - 1] = angle_rad[0]
        with viewer.lock():
            d.ctrl = data_ctrl
            for i in range(steps):
                mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(time_sleep)
        s_h += delta_s
