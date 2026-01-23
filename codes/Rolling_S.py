'''
说明：MuJoCo仿真实现Rolling S型横滚步态
参考文献：2009 Generating Gaits for Snake Robots by Annealed Chain Fitting and Keyframe Wave Extraction
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
N = 20              # 关节数量
l = 70

a1 = 40
a2 = 40
omega1 = 0.5
omega2 = 0.5
Omega1 = 0.2
Omega2 = 0.2
delta_1 = 0
delta_2 = 0
delta = math.pi / 2

n_joints = 10
gait_t = 0

control_angle = list(np.zeros(N))
data = np.zeros(N)

m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o20_Rolling_S.xml')
d = mujoco.MjData(m)


def RollingArc():
    global gait_t
    for i in range(20):
        for j in range(1, N + 1):
            # 原论文没有提及参数的选择问题，此处按笔者理解编写（原来论文中的l指关节编号，不是link length）
            if j % 2 == 1:
                # angle_rad = a1 / rad * np.sin(Omega1 * l + delta_1) * np.sin(omega1 * gait_t)
                angle_rad = a1 / rad * np.sin((int)(j / 2) * 2 * math.pi / n_joints) * np.sin(omega1 * gait_t)
            else:
                # angle_rad = a2 / rad * np.sin(Omega2 * l + delta_2) * np.sin(omega2 * gait_t + delta)
                angle_rad = a2 / rad * np.sin((int)(j / 2) * 2 * math.pi / n_joints) * np.sin(omega2 * gait_t + delta)
            data[j - 1] = angle_rad
            with viewer.lock():
                d.ctrl = data
                for i in range(6):
                    mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(0.01)
        gait_t += 1

with mujoco.viewer.launch_passive(m, d) as viewer:
    while True:
        RollingArc()


