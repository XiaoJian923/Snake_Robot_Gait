'''
说明：MuJoCo仿真实现S-足波步态
参考文献：2024 A Unified Motion Modeling Approach for Snake Robots Gaits Generated with Backbone Curve Method
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923
时间：20250501
'''
import math
import time
import mujoco
import bisect
import numpy as np
import mujoco.viewer
from scipy import integrate

# 这条蛇的关节轴之间的长度好像是70？
l = 0.064
r1 = 0.11
r2 = 0.11

beta = 2*math.atan(r1/r2)
L = 2*r1*math.pi + 2*r2*beta
# print(beta)
# print(L)

# Psi_array = [-math.pi/2, math.pi/2, math.pi/2, -math.pi/2]
Psi_array = [0, math.pi/2, math.pi/2, -math.pi/2]

# 计算6个简单曲线段长度的累加，用于定位弧长s处于第几个简单曲线段 直线为特殊的曲线
sum = 0
length_array = [r1*math.pi, r2*beta, r1*math.pi, r2*beta]
accumulate_array = []
for i in length_array:
    sum += i
    accumulate_array.append(sum)
# print(sum)

# 计算在弧长s下，当前正处于第几个简单曲线段
def calc_n(s):
    n = bisect.bisect_left(accumulate_array, s%sum)
    return n + 1

# 计算扭转角 (8)
def Psi(s):
    Psi_0 = 0
    temp = 0
    n = calc_n(s)
    for i in range(n):
        temp += Psi_array[i]
    return Psi_0 + temp

# 判断当前弧长s处于第几个简单曲线段，并返回对应曲率 TABLE Ⅰ
def K(s):
    n = calc_n(s)
    # if n in [1, 4]:
    if n in [1, 4]:
        return 1/r1
    return 1/r2

# 返回当前弧长s所对应的挠率 TABLE Ⅰ
def T(s):
    return 0

# 俯仰关节对应曲率 (1)
def Kp(s):
    return -1 * K(s) * math.sin(Psi(s))


# 偏航关节对应曲率 (2)
def Ky(s):
    return K(s) * math.cos(Psi(s))


s_h = 0         # 距离头部关节前段的位置，改变s_h的值（如递增操作），即可实现shift control
delta_s = l    # 实现shift control时每次递增的增量，这里取关节长度
N = 40          # 蛇形机器人的关节数量

time_sleep = 0.05   # 每epoch睡眠时间
epochs = 800        # 循环次数 800
steps = 2000         # step次数



m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o40.xml')   # MjModel包含了模型描述, 比如所有均不随时间变化的量
d = mujoco.MjData(m)
data_ctrl = np.zeros(N)


with mujoco.viewer.launch_passive(m, d) as viewer:
    # time.sleep(15)
    for i in range(epochs):
        for j in range(1, N+1): # 计算关节角度 (4)
            if j % 2 == 0:
                angle_rad = integrate.quad(Kp, s_h + (j - 1) * l, s_h + (j + 1) * l, limit=200)
            else:
                angle_rad = integrate.quad(Ky, s_h + (j - 1 ) *l, s_h + (j + 1) * l, limit=200)
            data_ctrl[j-1] = angle_rad[0]
        # print('desired angle =', data_ctrl)
        with viewer.lock():
            d.ctrl = data_ctrl
            for i in range(steps):
                mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(time_sleep)
        s_h += delta_s
        # print('actual angle=', d.sensordata[0:40])
