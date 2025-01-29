'''
说明：MuJoCo仿真实现履带步态。存在很多细节待完善
参考文献：2016 Gait design of a snake robot by connecting simple shapes
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑
时间：20250129
'''
import math
import time
import mujoco
import bisect
import numpy as np
import mujoco.viewer
from scipy import integrate


l = 80      # 关节长度
h = 200     # 履带高
w = 250     # 履带宽
d = 160     # 履带距离

r_c = math.sqrt(h**2+(w/2)**2)/2        # 圆弧半径，由勾股定理可得 (18)
alpha = 2*math.atan(w/(2*h))            # 圆弧之间扭转角 (19)

# Fixme:扭转角，见论文TABLE Ⅲ. 列表中元素类型为 int 或 float
# Fixme:论文在这里是不是写得不太好？笔者见论文着重强调圆弧之间的扭转角，但貌似没怎么提及直线与圆弧之间的扭转角。下述列表【1】对应仅考虑圆弧之间扭转角，列表【2】【3】对应圆弧之间加
# Fixme:圆弧与直线之间扭转角。PS：此处不排除是笔者对论文理解有误。
# Psi_array = [0, alpha, 0, 0, -alpha, 0]   # 【1】
# Psi_array = [0, -alpha/2, alpha, -alpha/2, -alpha/2, -alpha] # 【2】
Psi_array = [0, alpha/2, -alpha, alpha/2, alpha/2, alpha] # 【3】

l_j = 2 * r_c + d   # 直线长度
r_j = r_c           # 圆弧半径
Phi_j = math.pi     # 圆弧中心角：由圆的两条半径形成的角度

# 计算6个简单曲线段长度的累加，用于定位弧长s处于第几个简单曲线段 直线为特殊的曲线
sum = 0
length_array = [l_j, Phi_j*r_j, Phi_j*r_j, l_j, Phi_j*r_j, Phi_j*r_j]
accumulate_array = []
for i in length_array:
    sum += i
    accumulate_array.append(sum)
# sum = 2494

# 计算在弧长s下，当前正处于第几个简单曲线段
def calc_n(s):
    n = bisect.bisect_left(accumulate_array, s%sum)
    return n + 1


# 计算扭转角 (8)
# 由于构成履带步态Crawler Gait的各曲线挠率均为0，因此省去了对挠率的积分项
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
    if n in [1, 4]:
        return 0
    return 1/r_j


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
delta_s = 80    # 实现shift control时每次递增的增量，这里取关节长度
N = 30          # 蛇形机器人的关节数量

time_sleep = 0.01   # 每epoch睡眠时间
epochs = 800        # 循环次数
steps = 350         # step次数

m = mujoco.MjModel.from_xml_path('XJ30.xml')
d = mujoco.MjData(m)
data_ctrl = np.zeros(N)

with mujoco.viewer.launch_passive(m, d) as viewer:
    # time.sleep(15)
    for i in range(epochs):
        for j in range(1, N+1): # 计算关节角度 (4)
            if j % 2 == 1:
                angle_rad, _ = integrate.quad(Kp, s_h + (j - 1) * l, s_h + (j + 1) * l)
            else:
                angle_rad, _ = integrate.quad(Ky, s_h + (j - 1 ) *l, s_h + (j + 1) * l)
            data_ctrl[j-1] = angle_rad
        with viewer.lock():
            d.ctrl = data_ctrl
            for i in range(steps):
                mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(time_sleep)
        s_h += delta_s
        # print(data_ctrl)
