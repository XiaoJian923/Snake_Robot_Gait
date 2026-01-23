'''
说明：MuJoCo仿真实现从地面Rolling Arc横滚过渡到爬竖直杆子。本代码具有交互性。当a=-76，v=0.05时可以实现爬杆子。
注意，以上参数值仅供参考，并非标准。
参考文献：2013 Motion Estimation of Snake Robots in Straight Pipes
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923
时间：20260122
'''
import math
import time
import queue
import mujoco
import keyboard
import threading
import numpy as np
import mujoco.viewer


NUM_JOINTS = 20
angle_rad = 57.2958


# 初始化RollingArc步态参数  全局变量
a = 30
v = 0.0
omega = 1.57
time_sleep = 0.01
time_step = 0.3
step = 1
y = 0.0
t = 0.0

'''
通过按键来调整参数值a和v，↑增大a，↓减小a，←减小v，→增大v，f5反向[单步调试]，f6正向[单步调试]
'''
def increase_a():
    global a
    a += 1
    print(f"a增加到: {a}")

def decrease_a():
    global a
    a -= 1
    print(f"a减少到: {a}")

def increase_v():
    global v
    v += 0.01
    print(f"v增加到: {v}")

def decrease_v():
    global v
    v -= 0.01
    print(f"v减少到: {v}")

def reverse_single_step():
    global step, time_step
    time_step = -abs(time_step)
    step += 1
    print(f"step = {step}, time_step < 0")

def single_step():
    global step, time_step
    time_step = abs(time_step)
    step += 1
    print(f"step = {step}, time_step > 0")

# 注册热键 ，，，
keyboard.add_hotkey('up', increase_a)           # ↑增大a
keyboard.add_hotkey('down', decrease_a)         # ↓减小a
keyboard.add_hotkey('right', increase_v)        # ←减小v
keyboard.add_hotkey('left', decrease_v)         # →增大v
keyboard.add_hotkey('f5', reverse_single_step)   # 反向[单步调试]
keyboard.add_hotkey('f6', single_step)           # 正向[单步调试]

'''
通过字符串快速设置参数
'''
# 创建命令队列
command_queue = queue.Queue()

def uart_listener():
    """模拟串口监听线程 - 相当于中断服务程序"""
    while True:
        try:
            # 这里模拟从串口接收数据
            # 实际使用时可以替换为 pyserial 的读取
            user_input = input("Enter command, please!")
            command_queue.put(user_input)
            time.sleep(0.1)  # 防止过度占用CPU
        except KeyboardInterrupt:
            break

def command_processor():
    """命令处理线程 - 相当于中断处理"""
    global a, v, time_sleep, time_step, step

    while True:
        try:
            # 从队列获取命令（非阻塞）
            command = command_queue.get(timeout = 0.1)

            # 解析命令并修改变量
            if '=' in command:
                for subcommand in command.split():
                    key, value = subcommand.split('=')
                    key = key.strip().lower()

                    if key == 'a':
                        a = float(value)
                    elif key == 'v':
                        v = float(value)
                    elif key == 'time_sleep':
                        time_sleep = float(value)
                    elif key == 'time_step':
                        time_step = float(value)
                    elif key == 'step':
                        step = float(value)
                    else:
                        print(f"未知命令: {subcommand}")
                print(f"[参数更新] a = {a}, v = {v}, time_sleep = {time_sleep}, time_step = {time_step}, step = {step}")

            command_queue.task_done()

        except queue.Empty:
            continue
        except ValueError as e:
            print(f"命令格式错误: {e}")
        except KeyboardInterrupt:
            break

# 启动监听线程
listener_thread = threading.Thread(target=uart_listener, daemon=True)
processor_thread = threading.Thread(target=command_processor, daemon=True)

listener_thread.start()
processor_thread.start()

steps = 10  # step次数

m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o20_Pole_Climbing.xml')
d = mujoco.MjData(m)
data_ctrl = np.zeros(NUM_JOINTS)


with mujoco.viewer.launch_passive(m, d) as viewer:
    while(step > 0):
        for i in range(NUM_JOINTS):
            if (i % 2 == 0):
                y = a / angle_rad * math.sin(omega * t + v * i)
            else:
                y = a / angle_rad * math.sin(omega * t + v * i + math.pi / 2)
            data_ctrl[i] = y
            with viewer.lock():
                d.ctrl = data_ctrl
                for k in range(steps):
                    mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(time_sleep)
        t = t + time_step
        # step = step - 1



