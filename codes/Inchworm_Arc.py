'''
说明：MuJoCo仿真实现弧形尺蠖步态
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
n_joints = 8
omega = 0.8185
a = 45              # 最大幅值45
c = 0
direct = -2
delta_t = 0.1
gait_t = 0

control_angle = list(np.zeros(N))
data = np.zeros(N)

m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o14_Inchworm_Arc.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    for i in range(200):
        for j in range(1, N + 1):
            if j % 2 == 0:
                if j <= N / 2:
                    angle_rad = 0.4
                else:
                    angle_rad = 0.4
            else:
                angle_rad = a / rad * np.sin(omega * np.pi * 2 * gait_t + direct * (int)(j / 2) * 2 * math.pi / n_joints)
            data[j - 1] = angle_rad
            with viewer.lock():
                d.ctrl = data
                for i in range(6):
                    mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(0.01)
        gait_t += delta_t
