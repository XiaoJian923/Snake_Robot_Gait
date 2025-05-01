'''
说明：MuJoCo仿真实现行波步态。可视为沿蛇身偏转90度的蜿蜒步态
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923
时间：20250501
'''
import time
import math
import mujoco
import numpy as np
import mujoco.viewer

rad = 57.2957795    # 1弧度约等于57.2957795角度
N = 30              # 关节数量
n_joints = 8
omega = 0.8185
a = 45              # 最大幅值
c = 0
direct = -1
delta_t = 0.1
gait_t = 0

control_angle = list(np.zeros(N))
data = np.zeros(N)

m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o30.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    for i in range(200):
        for j in range(1, N+1):
            if j % 2 == 0:          # 与蜿蜒步态代码唯一不同的地方
                angle_rad = 0
            else:
                angle_rad = a / rad * np.sin(omega * np.pi * 2 * gait_t + direct * (int)(j/2)*2*math.pi/n_joints)
            data[j-1] = angle_rad
            with viewer.lock():
                d.ctrl = data
                for i in range(6):
                    mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(0.01)
        gait_t += delta_t
