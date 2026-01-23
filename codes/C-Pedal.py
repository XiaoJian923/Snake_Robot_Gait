'''
说明：MuJoCo仿真实现C-足波步态运动。本代码包括仿真代码与实机代码。运行仿真代码，待初始化完成后，键盘键入交互
命令，如step=200，即可看到仿真效果。实机代码适配飞特舵机SM-80BL-C001。
参考文献：2021 Hoop-Passing Motion for a Snake Robot to Realize Motion Transition Across Different Environments
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923
时间：20260122
'''
import time
import math
import queue
import serial
import bisect
import mujoco
import keyboard
import threading
import numpy as np
import mujoco.viewer
from scipy import integrate
import serial.tools.list_ports


#TODO：自定义异常
class MyException(BaseException):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return self.message + "__xj"


#TODO：获取所有串口设备实例
def SerialPort():
    xj = None
    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0:
        print("No Device")
    else:
        for comport in ports_list:
            if comport.description.find('CH340') != -1:
                xj = comport
        print("Available devices:", xj)

    #TODO：打开串口 读数据时延时timeout秒
    ser = serial.Serial(xj.device, 115200, timeout=0.1)
    if ser.is_open:  # 判断串口是否成功打开
        print(ser.name, "Open Success！")
    else:
        print(ser.name, "Open Failed!")
    return ser


#TODO：写入位置信息
def FT_Goal_Position(Ser, MotorID, Position_DEC):
    """Position_DEC是角度制"""
    Goal_Position = 'FF FF 01 09 03 2A 00 08 00 00 D0 07 FF'    # 最后这个FF是占位符，是校验位
    Goal_Position_Array = bytearray.fromhex(Goal_Position)
    # 更新电机ID信息
    Goal_Position_Array[2] = MotorID & 0xFF
    # 更新位置信息
    Position = Position_DEC / 90.0 * 1024 + 2048
    for i in range(2):
        Goal_Position_Array[6 + i] = int(Position) >> i * 8 & 0xFF
    # 计算校验码并更新
    sum = 0
    for i in range(2, 12):
        sum += Goal_Position_Array[i]
    Goal_Position_Array[-1] = ~(sum & 0xFF) & 0xFF
    # 发送命令数组
    Ser.write(Goal_Position_Array)

# 串口
#TODO:仿真
ser = None
#TODO:实机
# ser = SerialPort()


#TODO：机器人构型参数
# 仿真
NUM_JOINTS = 40
l = 70
# 实机
# NUM_JOINTS = 36
# l = 65

#TODO：人机交互参数
step = 1                # 连续执行步数
time_sleep = 0.01        # 连续执行步数之间的睡眠时间 0.3
time_sleep_init = 0.01   # 步态初始化时各关节之间的睡眠时间 0.3
delta_s = l * 0.1       # 实现shift control时每次递增的增量，这里取关节长度的0.1倍

#TODO：其他参数
angle_rad = 57.2958     # 弧度制转角度值系数


'''
通过按键来调整参数值，f5反向[单步调试]，f6正向[单步调试]
'''
def reverse_single_step():
    global step, delta_s
    delta_s = -abs(delta_s)
    step += 1
    print(f"step = {step}, delta_s < 0")

def single_step():
    global step, delta_s
    delta_s = abs(delta_s)
    step += 1
    print(f"step = {step}, delta_s > 0")

# 注册热键 ，，，
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
    global time_sleep, step, ser

    while True:
        try:
            # 从队列获取命令（非阻塞）
            command = command_queue.get(timeout=0.1)

            if 'rerun' in command:
                ser = SerialPort()
                print("Reconnected!")

            # 解析命令并修改变量
            if '=' in command:
                for subcommand in command.split():
                    key, value = subcommand.split('=')
                    key = key.strip().lower()

                    if key == 'time_sleep':
                        time_sleep = float(value)
                    elif key == 'step':
                        step = float(value)
                    else:
                        print(f"未知命令: {subcommand}")
                print(f"[参数更新] time_sleep = {time_sleep}, step = {step}")

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

# TODO：仿真
m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o40_C-Pedal.xml')
d = mujoco.MjData(m)
data_ctrl = np.zeros(NUM_JOINTS)

# TODO：仿真
def MuJoCo_Viewer():
    with viewer.lock():
        d.ctrl = data_ctrl
        for ii in range(50):
            mujoco.mj_step(m, d)
        viewer.sync()
        time.sleep(time_sleep)

'''************************************************以上与具体步态无关************************************************'''

# TODO：控制变量
s_h = 0.0       # shift control
Psi_0 = 0.0       # rolling control

# TODO：步态参数，由操作者定，可改变
h_p = 100
w_p = 250
d_p = 400
Ss = 10

alpha_p = 2 * math.atan(w_p / (2 * h_p))
a_p = math.sqrt(h_p ** 2 + (w_p / 2) ** 2)

r_p = (4 * a_p ** 2 + d_p ** 2) / (16 * a_p)
beta_p = math.atan(d_p / (4 * r_p - 2 * a_p))

l_pu = 4 * r_p * beta_p

print('alpha_p', alpha_p)
print('a_p', a_p)
print('r_p', r_p)
print('beta_p', beta_p)
# print('d_p', d_p)
print('l_pu', l_pu)

#TODO the twist angle between e2(s_j-) and e2(s_j+) around e1(s_j-)     开头是第一个片段与上个片段的扭转角
#TODO 将n=0，n=1的情况一次性列出
Psi_array = [alpha_p, math.pi, math.pi, -1 * alpha_p, math.pi, math.pi]
r_j = r_p           # 圆弧半径
Phi_j = beta_p     # 圆弧中心角：由圆的两条半径形成的角度

# 计算6个简单曲线段长度的累加，用于定位弧长s处于第几个简单曲线段 直线为特殊的曲线
sum = 0
length_array = [r_j * Phi_j, r_j * 2 * Phi_j, r_j * Phi_j, r_j * Phi_j, r_j * 2 * Phi_j, r_j * Phi_j]
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
    n = bisect.bisect_left(accumulate_array, s%sum)
    return n + 1


# 计算扭转角 (8)
# 嵌套积分
def Psi(s):
    n = calc_n(s)
    temp = (s//sum) * (4 * math.pi)
    for i in range(n):
        temp += Psi_array[i]
    return Psi_0 + temp


# 判断当前弧长s处于第几个简单曲线段，并返回对应曲率
def K(s):
    return 1/r_p


# 返回当前弧长s所对应的挠率 TABLE Ⅰ
def T(s):
    return 0


# 俯仰关节对应曲率 (1)
def Kp(s):
    if s < 0:
        return 0
    return -1 * K(s) * math.sin(Psi(s))


# 偏航关节对应曲率 (2)
def Ky(s):
    if s < 0:
        return 0
    return K(s) * math.cos(Psi(s))


def C_Pedal(time_init = 0.0, Init = False):
    global step, s_h, Psi_0
    while(step > 0):
        for j in range(1, NUM_JOINTS+1): # 计算关节角度 (4)
            if j % 2 == 1:
                y = integrate.quad(Kp, s_h - (j + 1) * l, s_h - (j - 1) * l)[0]
            else:
                y = integrate.quad(Ky, s_h - (j + 1) * l, s_h - (j - 1) * l)[0]
            # zzy.append(y)
            if y <=-89.8 or y >= 89.8:
                print(f"j = {j}, y = {y}")
                print("**********************************")
                while 1:
                    xj = 1

            # 判断是否为初始构型，若为初始构型则分n步渐变
            if Init:    # 初始化构型
                # 分n步的渐变程序
                for i_y in np.linspace(0, y, 2)[1:]:# TODO: y = 0 时不要走这里
                    time.sleep(time_init)
                    # 实机程序
                    # FT_Goal_Position(ser, j, i_y * angle_rad)
                    # 仿真程序
                    data_ctrl[j - 1] = i_y
                    MuJoCo_Viewer()
            else:
                time.sleep(time_init)
                # 实机程序
                # FT_Goal_Position(ser, j, y * angle_rad)
                # 仿真程序
                data_ctrl[j - 1] = y
        MuJoCo_Viewer()
        time.sleep(time_sleep)
        s_h += delta_s
        step = step - 1
        if step == 0:
            print("Now the s_h = ", s_h)
            print("Now the Psi_0 = ", Psi_0)


# TODO：仿真
with mujoco.viewer.launch_passive(m, d) as viewer:
    print()
    print("initialize ... ")
    C_Pedal(time_init = time_sleep_init, Init = True)      # 初始化，将step=1消耗掉
    print("initialize ... Over!")

    while True:
        C_Pedal()


# TODO：实机
# print()
# print("initialize ... ")
# C_Pedal(time_init = time_sleep_init, Init = True)      # 初始化，将step=1消耗掉
# print("initialize ... Over!")
# while True:
#     try:
#         C_Pedal()
#     except:
#         print("The final value of gait_t is ", gait_t)

