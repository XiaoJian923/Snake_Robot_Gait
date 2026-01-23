'''
说明：MuJoCo仿真实现Sidewinding步态
参考文献：2009 Parameterized and Scripted Gaits for Modular Snake Robots
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923
时间：20260122
'''
import time
import math
import mujoco
import numpy as np
import mujoco.viewer

rad = 57.2957795    # 1弧度约等于57.2957795角度
N = 14              # 关节数量
omega = 1.5185
a1 = 60              # 横向和纵向的幅值
a2 = 30     #25
xj = 0.58
delta_t = 0.22
gait_t = 0

control_angle = list(np.zeros(N))
data = np.zeros(N)

m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o14_Sidewinding.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    for i in range(800):
        for j in range(1, N + 1):
            if j % 2 == 0:
                angle_rad = a1 / rad * np.sin(omega * gait_t + j * xj)
            else:
                angle_rad = a2 / rad * np.sin(omega * gait_t + j * xj+ np.pi / 2)
            data[j - 1] = angle_rad
            with viewer.lock():
                d.ctrl = data
                for i in range(6):
                    mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(0.01)
        gait_t += delta_t
