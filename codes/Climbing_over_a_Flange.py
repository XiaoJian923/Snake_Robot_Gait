'''
说明：MuJoCo仿真实现蛇形机器人爬法兰盘构型
参考文献：2018 Gait Design for a Snake Robot by Connecting Curve Segments and Experimental Demonstration
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923
时间：20260121
'''
import math
import time
import mujoco
import bisect
import numpy as np
import mujoco.viewer
from scipy import integrate

#TODO：自定义异常
class MyException(BaseException):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return self.message + "__xj"


#TODO：机器人构型参数
NUM_JOINTS = 40     # 关节数量
l = 65              # 连杆长度
diameter_link = 60  # 连杆直径

delta_s = l * 0.1       # 实现shift control时每次递增的增量，这里取关节长度的0.1倍

#TODO：其他参数
angle_rad = 57.2958     # 弧度制转角度值系数
y = 0.0                 # 舵机角度变量

# TODO：控制变量
s_h = 0.0       # shift control
Psi_0 = 0       # rolling control

# TODO：步态参数，由环境决定
r_h = 95    # 管子的半径 + 蛇形机器人连杆的半径
h = 80       # bridge part 距离圆柱表面的距离
d = 210      # bridge part 上下两个平行segment的距离

# TODO：步态参数，由操作者定
r_c = 90  # the radius of all circular arcs   85以上可以，80就不行
p_h = 110    # 螺距
slope = p_h / (2 * math.pi)   # 螺旋线的slope

phi = (NUM_JOINTS + 1) * l / math.sqrt(r_h ** 2 + slope ** 2)  # 螺旋线的中心角度

print()
# TODO：由几何推导出的参数
alpha = math.atan(slope / r_h)
h_min = math.sqrt(r_h ** 2 + r_c ** 2 * (1 + math.sin(alpha)) ** 2) - r_h

Angle_OhOcV = math.acos(((r_h + r_c) ** 2 + (r_c * math.sin(alpha)) ** 2 + 2 * r_c ** 2 - (r_h + h) ** 2) / (2 * math.sqrt(2) * r_c * math.sqrt((r_h + r_c) ** 2 + (r_c * math.sin(alpha)) ** 2)))
Angle_OhVOc = math.acos((2 * r_c ** 2 + (r_h + h) ** 2 - (r_h + r_c) ** 2 - (r_c * math.sin(alpha)) ** 2) / (2 * math.sqrt(2) * r_c * (r_h + h)))
print("Angle_OhOcV:", Angle_OhOcV * angle_rad)
print("Angle_OhVOc:", Angle_OhVOc * angle_rad)
beta = Angle_OhOcV - math.atan(r_c * math.sin(alpha) / (r_h + r_c)) - (math.pi / 4)
gamma = math.pi * 5 / 2 - 2 * Angle_OhVOc
# TODO：以上四个公式没问题，已验证
l_s = 0
delta_twist = -2 * math.tan(alpha) / diameter_link * delta_s    # TODO：扭转角delta_twist与shift control delta_s的数量关系

# 螺旋线的曲率和挠率
k_helix = r_h / (r_h ** 2 + slope ** 2)
tau_helix = slope / (r_h ** 2 + slope ** 2)

print("管子的半径r_h：", r_h)
print("bridge part 距离管道表面的距离h：", h)
print("bridge part 上下两个平行片段的距离d：", d)
print("片段中圆弧的半径r_c：", r_c)
print("螺旋曲线螺距p_h：", p_h)
print("螺旋曲线的slope:", slope)
print("螺旋曲线的中心角度phi：", phi * angle_rad)
print("螺旋曲线与倾角alpha：", alpha * angle_rad)
print("圆弧片段3的圆心角beta：", beta * angle_rad)
print("直线片段4的长度l_s：", l_s)
print("中间片段的扭转角gamma：", gamma * angle_rad)
print("bridge part 距离管道表面的最小距离h_min：", h_min)
print("扭转角增量delta_twist：", delta_twist)
print("螺旋线的曲率k_helix：", k_helix)
print("螺旋线的挠率tau_helix：", tau_helix)


Psi_array = [math.pi / 2, -math.pi / 2, -math.pi / 2, 0, -math.pi / 2, 0, gamma, 0, math.pi / 2, math.pi / 2]


# 计算简单曲线段长度的累加，用于定位弧长s处于第几个简单曲线段
sum = 0
length_array = [phi * math.sqrt(r_h ** 2 + slope ** 2),
                r_c * alpha,
                r_c * beta,
                l_s,
                r_c * math.pi / 2,
                d - 2 * r_c,
                r_c * math.pi / 2,
                l_s,
                r_c * beta,
                r_c * alpha]
# 检查length_array中的值是否有错误
try:
    for length in length_array:
        if length < 0:
            raise MyException("The length of the curve segments should be greater than or equal to 0.")
except MyException as e:
    print("Caught MyException:", e)

accumulate_array = []
for i in length_array:
    sum += i
    accumulate_array.append(sum)
print(length_array)
print(accumulate_array)

# 计算在弧长s下，当前正处于第几个简单曲线段
def calc_n(s):
    n = bisect.bisect_left(accumulate_array, s % sum)
    return n + 1


# 计算扭转角
# 嵌套积分
def Psi(s):
    n = calc_n(s)
    #对挠率进行积分
    twist_helix = length_array[0] * tau_helix   # 螺旋线整体挠率
    temp = (s // sum) * (twist_helix + gamma - 3 * math.pi)
    if n == 1:
        temp += (s % sum) * tau_helix
    else:
        temp += twist_helix

    for i in range(n):
        temp += Psi_array[i]
    return Psi_0 + temp


# 判断当前弧长s处于第几个简单曲线段，并返回对应曲率
def K(s):
    n = calc_n(s)
    if n == 1:
        return k_helix
    elif n in [4, 6, 8]:
        return 0
    return 1 / r_c


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


time_sleep = 0.01   # 每epoch睡眠时间
epochs = 3800        # 循环次数
steps = 50         # step次数

m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o40_Climbing_over_a_Flange.xml')
d = mujoco.MjData(m)
data_ctrl = np.zeros(NUM_JOINTS)

with mujoco.viewer.launch_passive(m, d) as viewer:
    for i in range(epochs):
        for j in range(1, NUM_JOINTS + 1): # 计算关节角度
            if j % 2 == 0:
                y = integrate.quad(Kp, s_h - (j + 1) * l, s_h - (j - 1) * l)[0]
            else:
                y = integrate.quad(Ky, s_h - (j + 1) * l, s_h - (j - 1) * l)[0]
            data_ctrl[j - 1] = y
        with viewer.lock():
            d.ctrl = data_ctrl
            for ii in range(steps):
                mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(time_sleep)
        s_h += delta_s

