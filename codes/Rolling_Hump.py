"""
说明：MuJoCo仿真实现驼峰翻滚运动
参考文献：2015 Modeling Rolling Gaits of a Snake Robot
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923
时间：20260123
"""
import math
import time
import mujoco
import numpy as np
import mujoco.viewer
from tqdm import tqdm
from scipy import integrate
from scipy.integrate import quad
from scipy.optimize import newton

N = 20          # 蛇形机器人的关节数量
# 参数定义
r, h, theta0, sigma = 0.4, 0.13, np.pi * (2 / 4), 0.25

# 曲线导数
def dz_dtheta(theta):
    return -2 * h * (theta - theta0) / sigma ** 2 * np.exp(-(theta - theta0) ** 2 / sigma ** 2)

def d2z_dtheta2(theta):
    return (2 * h / sigma ** 2) * (2 * (theta - theta0) ** 2 / sigma ** 2 - 1) * np.exp(-(theta - theta0) ** 2 / sigma ** 2)

def d3z_dtheta3(theta):
    return (4 * h * (theta - theta0) / sigma ** 4) * (3 - 2 * (theta - theta0) ** 2 / sigma ** 2) * np.exp(-(theta - theta0) ** 2 / sigma ** 2)

# 弧长微分
def ds_dtheta(theta):
    dx = -r * np.sin(theta)
    dy = r * np.cos(theta)
    dz = dz_dtheta(theta)
    return np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

# 数值求解θ(s)
def theta_at_s(s_target, theta_start = 0, theta_end = 2 * np.pi):
    def arc_length(theta):
        return quad(ds_dtheta, theta_start, theta)[0]
    return newton(lambda theta: arc_length(theta) - s_target, x0 = (theta_start + theta_end) / 2)

# 计算κ和τ
def curvature_and_torsion(theta):
    dr = np.array([-r * np.sin(theta), r * np.cos(theta), dz_dtheta(theta)])
    ddr = np.array([-r * np.cos(theta), -r * np.sin(theta), d2z_dtheta2(theta)])
    dddr = np.array([r * np.sin(theta), -r * np.cos(theta), d3z_dtheta3(theta)])
    dr_x_ddr = np.cross(dr, ddr)
    kappa = np.linalg.norm(dr_x_ddr) / (np.linalg.norm(dr) ** 3)
    tau = np.dot(dr_x_ddr, dddr) / (np.linalg.norm(dr_x_ddr) ** 2 + 1e-10)
    return kappa, tau


l = 0.064
Psi_0 = 0


# 返回当前弧长s所对应的挠率
def T(s):
    theta_s = theta_at_s(s)
    return curvature_and_torsion(theta_s)[1]


# 因为要对0到s进行挠率积分，因此提前计算一部分
delta = l
buffer = []
t = 0
for xj in tqdm(range(1, N + 5)):
    t += integrate.quad(T, (xj - 1) * delta, xj * delta, limit = 200)[0]
    buffer.append(t)
print('num:', len(buffer))


# 计算扭转角
def Psi(s):
    temp = 0
    n = int(s // delta)
    if n > 0:
        temp = buffer[n - 1]
    temp += integrate.quad(T, n * delta, s, limit = 200)[0]
    return Psi_0 + temp


def T_integrate(s):
    return integrate.quad(T, 0, s, limit = 200)[0]


def K_cos(s):
    theta_s = theta_at_s(s)
    kappa_cos, tau_cos = curvature_and_torsion(theta_s)
    return kappa_cos * math.cos(Psi(s))


def K_sin(s):
    theta_s = theta_at_s(s)
    kappa_sin, tau_sin = curvature_and_torsion(theta_s)
    return kappa_sin * math.sin(Psi(s))


s_h = 0         # 距离头部关节前段的位置，改变s_h的值（如递增操作），即可实现shift control
delta_s = l    # 实现shift control时每次递增的增量，这里取关节长度
time_sleep = 0.05   # 每epoch睡眠时间
epochs = 800        # 循环次数 800
steps = 200         # step次数
m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o20_Rolling_Hump.xml')
d = mujoco.MjData(m)
data_ctrl = np.zeros(N)

# TODO:new
# alpha_cos = []
# alpha_sin = []
# for j in tqdm(range(1, N + 1)):
#     alpha_cos.append(integrate.quad(K_cos, (j - 1) * l, (j + 1) * l, limit = 200)[0])
#     alpha_sin.append(integrate.quad(K_sin, (j - 1) * l, (j + 1) * l, limit = 200)[0])
#
# print(alpha_cos)
# print(alpha_sin)

# 下面的两行，由上面注释的内容（114-121）所生成。
alpha_cos = [0.32, 0.3199999999999999, 0.31999999999713313, 0.3199999855723507, 0.3199865029372798, 0.31778003461336624, 0.2761350964818704, 0.1813881756705463, 0.18512016153139307, 0.5516648928753052, 0.5584655080570268, 0.1915034249607832, 0.17841955476514842, 0.2732419094823691, 0.3174349759936917, 0.31998335771066916, 0.3199999811192236, 0.31999999999602763, 0.3199999999999996, 0.32000000000000023]
alpha_sin = [1.7509786801696613e-10, 6.127613385416408e-08, 9.249444408371306e-06, 0.0005972989875932789, 0.016266879341519117, 0.17843759210890903, 0.6101854767416776, 0.6709823906980664, 0.031007250281582598, -1.3494718885612733, -1.3737659694831763, -0.005730500431372647, 0.6604206187941075, 0.6224939298001125, 0.19049500124572905, 0.01797964082888324, 0.0006808083060035108, 1.0863963748171451e-05, 7.393086551700538e-08, 7.935861906641858e-12]


# TODO:new
zz = 1
with mujoco.viewer.launch_passive(m, d) as viewer:
    for i in range(epochs):
        for j in range(1, N + 1):
            if j % 2 == 0:
                angle_rad = math.cos(Psi_0) * alpha_cos[j - 1] - math.sin(Psi_0) * alpha_sin[j - 1]
            else:
                angle_rad = math.sin(Psi_0) * alpha_cos[j - 1] + math.cos(Psi_0) * alpha_sin[j - 1]
            data_ctrl[j - 1] = angle_rad

        with viewer.lock():
            d.ctrl = data_ctrl
            for ii in range(steps):
                mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(time_sleep)
        if i > 0:
            Psi_0 -= math.pi / 25


