'''
说明：MuJoCo仿真实现爬梯子步态。此步态构型较复杂，上实机会更直观。
参考文献：2018 Ladder Climbing with a Snake Robot
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


l = 65      # the link length
angle_rad = 57.2958
#TODO：步态构型由4个参数所决定      r_c越大越平滑，越小每个step所需长度就越小
h_s = 260    # vertical interval between the steps of the ladder
l_s = 0      # horizontal interval between the steps of the ladder
alpha = 120 / angle_rad # the angle between ladder steps and the body of snake robot
r_c = 90  # the radius of all circular arcs 89.1

k_max = 0.011  # the maximum curvature that the snake robot can achieve
beta = math.atan(l_s / (h_s - 2 * r_c))
K_n = 1
K_max = 1.1

s_h = 0.0         # 距离头部关节前段的位置，改变s_h的值（如递增操作），即可实现shift control
delta_s = l * 0.05    # 实现shift control时每次递增的增量
N = 40          # 蛇形机器人的关节数量

#TODO the twist angle between e2(s_j-) and e2(s_j+) around e1(s_j-)     开头是第一个片段与上个片段的扭转角
#TODO 将n=0，n=1的情况一次性列出
Psi_array = [alpha + math.pi / 2, 0, 0, alpha - math.pi / 2, -1 * math.pi / 2, math.pi / 2, -1 * (alpha + math.pi / 2), 0, 0, math.pi / 2 - alpha, math.pi / 2, -1 * math.pi / 2]

r_j = r_c   # the radius of all circular arcs
l_j = (h_s - 2 * r_c) / math.cos(beta) # the length of the straight line segment

sum = 0
length_array = [r_j * (math.pi / 2 + beta), l_j, r_j * (math.pi / 2 - beta), r_j * math.pi / 2, r_j * 2 * alpha, r_j * math.pi / 2,
                r_j * (math.pi / 2 + beta), l_j, r_j * (math.pi / 2 - beta), r_j * math.pi / 2, r_j * 2 * alpha, r_j * math.pi / 2]
accumulate_array = []
for i in length_array:
    sum += i
    accumulate_array.append(sum)

# 计算在弧长s下，当前正处于第几个简单曲线段
def calc_n(s):
    n = bisect.bisect_left(accumulate_array, s % sum)
    return n + 1


def Psi(s):
    Psi_0 = 0
    temp = 0
    n = calc_n(s)
    for i in range(n):
        temp += Psi_array[i]
    return Psi_0 + temp


# 判断当前弧长s处于第几个简单曲线段，并返回对应曲率
def K(s):
    n = calc_n(s)
    if n in [2, 8]:
        return 0
    elif n in [3, 4, 9, 10]:
        return 1 / (K_n * r_j)
    return 1 / r_j


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


time_sleep = 0.01   # 每epoch睡眠时间
epochs = 3800        # 循环次数
steps = 50         # step次数


m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o40_Ladder_Climbing.xml')
d = mujoco.MjData(m)
data_ctrl = np.zeros(N)

with mujoco.viewer.launch_passive(m, d) as viewer:
    for i in range(epochs):
        # calculate the Kn
        s_h_hat = s_h % sum
        if accumulate_array[1] < s_h_hat <= accumulate_array[2] or accumulate_array[7] < s_h_hat <= accumulate_array[8]:
            K_n = K_max
        elif accumulate_array[2] < s_h_hat <= accumulate_array[3]:
            K_n = (K_max * (accumulate_array[3] - s_h_hat) + s_h_hat - accumulate_array[2]) / length_array[3]
        elif accumulate_array[8] < s_h_hat <= accumulate_array[9]:
            K_n = (K_max * (accumulate_array[9] - s_h_hat) + s_h_hat - accumulate_array[8]) / length_array[9]
        else:
            K_n = 1

        for j in range(1, N + 1):
            if j % 2 == 0:
                angle_rad = integrate.quad(Kp, s_h - (j + 1) * l, s_h - (j - 1) * l)[0]
            else:
                angle_rad = integrate.quad(Ky, s_h - (j + 1) * l, s_h - (j - 1) * l)[0]
            data_ctrl[j - 1] = angle_rad
        with viewer.lock():
            d.ctrl = data_ctrl
            for i in range(steps):
                mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(time_sleep)
        s_h += delta_s

