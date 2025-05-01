'''
说明：MuJoCo仿真实现爬水平管子步态
参考文献：2013 Motion Estimation of Snake Robots in Straight Pipes
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923
时间：20250501
'''
import time
import mujoco
import numpy as np
import mujoco.viewer

N = 20              # 关节数
amplitude = 1.13    # 幅值越大，半径越小
omega = 4.5
v = 0.58
time_gait = 0
time_gait_incremental = 0.25
time_sleep = 0.01
epochs = 400  # 循环次数
steps = 10  # step次数

m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o20_pipe.xml')
d = mujoco.MjData(m)
data_ctrl = np.zeros(N)

with mujoco.viewer.launch_passive(m, d) as viewer:
    # time.sleep(20)
    for i in range(epochs):
        for j in range(N):
            if j % 2 == 0:
                angle_rad = amplitude * np.sin(omega * time_gait + v * j)
            else:
                angle_rad = amplitude * np.sin(omega * time_gait + v * j + np.pi / 2)
            data_ctrl[j] = angle_rad
            with viewer.lock():
                d.ctrl = data_ctrl
                for i in range(steps):
                    mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(time_sleep)
        time_gait += time_gait_incremental
