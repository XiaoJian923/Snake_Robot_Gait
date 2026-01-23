'''
说明：MuJoCo仿真实现履带步态转弯
参考文献：2018 Gait Design for a Snake Robot by Connecting Curve Segments and Experimental Demonstration
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923
时间：20260122
'''
import math
import time
import mujoco
import bisect
import numpy as np
import mujoco.viewer
from scipy import integrate

l = 70      # 关节长度
h = 175     # 履带高
w = 160     # 履带宽
d = 200     # 履带距离

r_c = math.sqrt(h ** 2 + (w / 2) ** 2) / 2    # 圆弧半径，由勾股定理可得
alpha = 2 * math.atan(w / (2 * h))            # 圆弧之间扭转角
r_t = 400

# 用于控制方向
a1 = -0.5 * math.pi
a2 = 0.5 * math.pi
a3 = 0
a4 = -0.5 * math.pi
a5 = 0.5 * math.pi
a6 = 0

# TODO：直线走
# Psi_array = [-alpha/2, alpha/2, alpha, alpha/2, -alpha/2, -alpha]
# TODO：逆时针走
# Psi_array = [-alpha / 2 + a1, alpha / 2 + a2, alpha + a3, alpha / 2 + a4, -alpha / 2 + a5, -alpha + a6]
# TODO：顺时针走
Psi_array = [-alpha / 2 - a1, alpha / 2 - a2, alpha + a3, alpha / 2 - a4, -alpha / 2 - a5, -alpha + a6]


l_j = 2 * r_c + d   # 直线长度
r_j = r_c           # 圆弧半径
Phi_j = math.pi     # 圆弧中心角：由圆的两条半径形成的角度
OU1 = r_t - w / 4
OU2 = r_t + w / 4
r_r = r_t + w / 2
Phi_r = 2 * math.atan(r_c / OU1) + 2 * math.asin(d / (2 * math.sqrt(OU1 * OU1 + r_c * r_c)))
r_l = r_t - w / 2
Phi_l = 2 * math.atan(r_c / OU2) + 2 * math.asin(d / (2 * math.sqrt(OU2 * OU2 + r_c * r_c)))

# 计算6个简单曲线段长度的累加，用于定位弧长s处于第几个简单曲线段
sum = 0
length_array = [r_r * Phi_r, Phi_j * r_j, Phi_j * r_j, r_l * Phi_l, Phi_j * r_j, Phi_j * r_j]
accumulate_array = []
for i in length_array:
    sum += i
    accumulate_array.append(sum)
print(accumulate_array)

# 计算在弧长s下，当前正处于第几个简单曲线段
def calc_n(s):
    n = bisect.bisect_left(accumulate_array, s % sum)
    return n + 1


# 计算扭转角
# 由于构成履带步态的各曲线挠率均为0，因此省去了对挠率的积分项
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
    if n == 1:
        return 1 / r_r
    if n == 4:
        return 1 / r_l
    return 1 / r_j


# 返回当前弧长s所对应的挠率
def T(s):
    return 0


# 俯仰关节对应曲率
def Kp(s):
    return -1 * K(s) * math.sin(Psi(s))


# 偏航关节对应曲率
def Ky(s):
    return K(s) * math.cos(Psi(s))


s_h = 0         # 距离头部关节前段的位置，改变s_h的值（如递增操作），即可实现shift control
delta_s = l * 0.5    # 实现shift control时每次递增的增量
N = 40          # 蛇形机器人的关节数量

time_sleep = 0.05   # 每epoch睡眠时间
epochs = 200        # 循环次数 800
steps = 500         # step次数

m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o40_Crawler_Turning_Form.xml')
d = mujoco.MjData(m)
data_ctrl = np.zeros(N)

with mujoco.viewer.launch_passive(m, d) as viewer:
    for i in range(epochs):
        for j in range(1, N + 1): # 计算关节角度
            if j % 2 == 1:
                angle_rad = integrate.quad(Kp, s_h + (j - 1) * l, s_h + (j + 1) * l)[0]
            else:
                angle_rad = integrate.quad(Ky, s_h + (j - 1 ) *l, s_h + (j + 1) * l)[0]

            data_ctrl[j - 1] = angle_rad
        with viewer.lock():
            d.ctrl = data_ctrl
            for j in range(steps):
                mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(time_sleep)
        s_h += delta_s


