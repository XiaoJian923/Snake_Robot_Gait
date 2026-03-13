"""
说明：升级代码结构，MuJoCo仿真实现履带步态
参考文献：2018 Gait Design for a Snake Robot by Connecting Curve Segments and Experimental Demonstration
MuJoCo模型参考：https://github.com/VanguardDream/snake_RL
作者：肖剑0923，Deepseek
时间：20260313
"""
import math
import time
import queue
import serial
import mujoco
import bisect
import keyboard
import threading
import numpy as np
import mujoco.viewer
from enum import Enum
from scipy import integrate
import serial.tools.list_ports
from typing import List, Tuple, Optional, Dict, Any


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


class CurveType(Enum):
    """曲线类型枚举"""
    LINE = "直线"
    ARC = "圆弧"
    HELIX = "螺旋线"


class CurveSegment:
    """曲线段基类"""

    def __init__(self, curve_type: CurveType, params: Dict[str, float]):
        """
        初始化曲线段

        Args:
            curve_type: 曲线类型
            params: 参数字典，不同曲线类型需要不同的参数
        """
        self.curve_type = curve_type
        self.params = params.copy()
        self.length = 0.0
        self._calculate_length()

    def _calculate_length(self):
        """计算曲线段长度，由子类实现"""
        raise NotImplementedError

    def get_kappa(self, s: float) -> float:
        """获取曲率，s为曲线段内的局部弧长"""
        raise NotImplementedError

    def get_tau(self, s: float) -> float:
        """获取挠率，s为曲线段内的局部弧长"""
        raise NotImplementedError

    def get_info(self) -> Dict[str, Any]:
        """获取曲线段信息"""
        info = {
            "type": self.curve_type.value,
            "length": self.length,
            "params": self.params.copy()
        }
        return info

    def update_params(self, new_params: Dict[str, float]):
        """更新参数并重新计算长度"""
        self.params.update(new_params)
        self._calculate_length()


class LineSegment(CurveSegment):
    """直线段"""

    def __init__(self, length: float):
        """
        初始化直线段

        Args:
            length: 直线长度
        """
        params = {"length": length}
        super().__init__(CurveType.LINE, params)

    def _calculate_length(self):
        self.length = self.params["length"]

    def get_kappa(self, s: float) -> float:
        return 0.0

    def get_tau(self, s: float) -> float:
        return 0.0


class ArcSegment(CurveSegment):
    """圆弧段"""

    def __init__(self, radius: float, angle: float):
        """
        初始化圆弧段

        Args:
            radius: 圆弧半径
            angle: 圆心角（弧度）
        """
        params = {"radius": radius, "angle": angle}
        super().__init__(CurveType.ARC, params)

    def _calculate_length(self):
        self.length = self.params["radius"] * abs(self.params["angle"])

    def get_kappa(self, s: float) -> float:
        # 圆弧曲率为半径的倒数
        return 1.0 / self.params["radius"]

    def get_tau(self, s: float) -> float:
        # 平面圆弧挠率为0
        return 0.0


class HelixSegment(CurveSegment):
    """螺旋线段"""

    def __init__(self, radius: float, pitch: float, turns: float):
        """
        初始化螺旋线段

        Args:
            radius: 螺旋线半径
            pitch: 螺距（每转一圈上升的高度）
            turns: 圈数
        """
        params = {"radius": radius, "pitch": pitch, "turns": turns}
        super().__init__(CurveType.HELIX, params)

    def _calculate_length(self):
        # 螺旋线长度 = sqrt((2πr)^2 + p^2) * turns
        r = self.params["radius"]
        p = self.params["pitch"]
        turns = self.params["turns"]

        # 一圈的长度
        one_turn_length = math.sqrt((2 * math.pi * r) ** 2 + p ** 2)
        self.length = one_turn_length * turns

    def get_kappa(self, s: float) -> float:
        # 螺旋线的曲率是常数
        r = self.params["radius"]
        p = self.params["pitch"]
        return r / (r ** 2 + (p / (2 * math.pi)) ** 2)

    def get_tau(self, s: float) -> float:
        # 螺旋线的挠率也是常数
        r = self.params["radius"]
        p = self.params["pitch"]
        return (p / (2 * math.pi)) / (r ** 2 + (p / (2 * math.pi)) ** 2)


class CurveSegments:
    """曲线段拼接类，用于生成蛇形机器人步态"""

    def __init__(self):
        """初始化曲线段拼接器"""
        self.segments: List[CurveSegment] = []
        self.accum_lengths: List[float] = []  # 累积长度
        self.total_length = 0.0
        self.psi_0 = 0.0  # 初始扭转角
        self.psi_array: List[float] = []  # 各段之间的扭转角

    def add_curve(self, curve: CurveSegment, position: Optional[int] = None,
                  psi: float = 0.0) -> bool:
        """
        添加曲线段

        Args:
            curve: 曲线段对象
            position: 插入位置（从0开始），None表示添加到末尾
            psi: 该曲线段起始处的扭转角（相对于前一曲线段末端）

        Returns:
            bool: 添加是否成功
        """
        if position is None:
            # 添加到末尾
            self.segments.append(curve)
            self.psi_array.append(psi)
        else:
            # 插入到指定位置
            if position < 0 or position > len(self.segments):
                print(f"错误：插入位置 {position} 超出范围 [0, {len(self.segments)}]")
                return False
            self.segments.insert(position, curve)
            self.psi_array.insert(position, psi)

        # 重新计算累积长度
        self._update_accum_lengths()
        return True

    # TODO：如果length=0，那么就将曲线移除
    def remove_curve(self, position: int) -> bool:
        """
        移除指定位置的曲线段

        Args:
            position: 要移除的曲线段索引

        Returns:
            bool: 移除是否成功
        """
        if position < 0 or position >= len(self.segments):
            print(f"错误：移除位置 {position} 超出范围 [0, {len(self.segments) - 1}]")
            return False

        self.segments.pop(position)
        self.psi_array.pop(position)
        self._update_accum_lengths()
        return True

    def _update_accum_lengths(self):
        """更新累积长度数组"""
        self.accum_lengths = []
        current_length = 0.0

        for seg in self.segments:
            current_length += seg.length
            self.accum_lengths.append(current_length)

        self.total_length = current_length if self.accum_lengths else 0.0

    def get_segment_count(self) -> int:
        """获取曲线段数量"""
        return len(self.segments)

    def get_segment_info(self, position: int) -> Optional[Dict[str, Any]]:
        """
        获取指定位置曲线段的信息

        Args:
            position: 曲线段索引

        Returns:
            曲线段信息字典，如果索引无效则返回None
        """
        if position < 0 or position >= len(self.segments):
            print(f"错误：索引 {position} 超出范围 [0, {len(self.segments) - 1}]")
            return None

        seg = self.segments[position]
        info = seg.get_info()
        info["psi"] = self.psi_array[position] if position < len(self.psi_array) else 0.0
        info["index"] = position

        return info

    def get_all_segments_info(self) -> List[Dict[str, Any]]:
        """获取所有曲线段的信息"""
        info_list = []
        for i, seg in enumerate(self.segments):
            info = seg.get_info()
            info["psi"] = self.psi_array[i] if i < len(self.psi_array) else 0.0
            info["index"] = i
            info_list.append(info)
        return info_list

    def update_segment_params(self, position: int, new_params: Dict[str, float]) -> bool:
        """
        更新指定位置曲线段的参数

        Args:
            position: 曲线段索引
            new_params: 新参数字典

        Returns:
            bool: 更新是否成功
        """
        if position < 0 or position >= len(self.segments):
            print(f"错误：索引 {position} 超出范围 [0, {len(self.segments) - 1}]")
            return False

        self.segments[position].update_params(new_params)
        # 上行代码已更新曲线长度信息，当长度为0时，可以删去该曲线
        if self.segments[position].length <= 0.01:
            self.remove_curve(position)
        self._update_accum_lengths()
        return True

    def update_segment_psi(self, position: int, psi: float) -> bool:
        """
        更新指定位置曲线段的扭转角

        Args:
            position: 曲线段索引
            psi: 新的扭转角（弧度）

        Returns:
            bool: 更新是否成功
        """
        if position < 0 or position >= len(self.segments):
            print(f"错误：索引 {position} 超出范围 [0, {len(self.segments) - 1}]")
            return False

        if position < len(self.psi_array):
            self.psi_array[position] = psi
        return True

    def find_segment(self, s: float) -> Tuple[int, float]:
        """
        根据全局弧长s找到对应的曲线段和局部弧长

        Args:
            s: 全局弧长

        Returns:
            (segment_index, local_s): 曲线段索引和在该段内的局部弧长
        """
        if not self.segments:
            return -1, 0.0

        # 归一化到总长度范围内
        s_normalized = s % self.total_length if self.total_length > 0 else 0

        # 找到对应的段
        idx = bisect.bisect_left(self.accum_lengths, s_normalized)

        if idx == 0:
            # 在第一段
            return 0, s_normalized
        elif idx < len(self.accum_lengths):
            # 在第idx段
            prev_length = self.accum_lengths[idx - 1]
            return idx, s_normalized - prev_length

    def get_psi(self, s: float) -> float:
        """
        计算全局弧长s处的扭转角

        Args:
            s: 全局弧长

        Returns:
            扭转角（弧度）
        """
        if not self.segments:
            return self.psi_0

        s_normalized = s % self.total_length if self.total_length > 0 else 0

        # 找到所在的段
        idx = bisect.bisect_left(self.accum_lengths, s_normalized)

        # 累积之前所有段的扭转角
        total_psi = self.psi_0
        for i in range(idx):
            if i < len(self.psi_array):
                total_psi += self.psi_array[i]

        return total_psi

    def get_kappa(self, s: float) -> float:
        """
        获取全局弧长s处的曲率

        Args:
            s: 全局弧长

        Returns:
            曲率值
        """
        if not self.segments:
            return 0.0

        idx, local_s = self.find_segment(s)
        if idx >= 0:
            return self.segments[idx].get_kappa(local_s)
        return 0.0

    def get_tau(self, s: float) -> float:
        """
        获取全局弧长s处的挠率

        Args:
            s: 全局弧长

        Returns:
            挠率值
        """
        if not self.segments:
            return 0.0

        idx, local_s = self.find_segment(s)
        if idx >= 0:
            return self.segments[idx].get_tau(local_s)
        return 0.0

    def get_pitch_curvature(self, s: float) -> float:
        """
        计算俯仰关节曲率 Kp(s) = -K(s) * sin(Psi(s))

        Args:
            s: 全局弧长

        Returns:
            俯仰关节曲率
        """
        if s < 0:
            return 0
        k = self.get_kappa(s)
        psi = self.get_psi(s)   # 这里少积分项，要更改
        return -k * math.sin(psi)

    def get_yaw_curvature(self, s: float) -> float:
        """
        计算偏航关节曲率 Ky(s) = K(s) * cos(Psi(s))

        Args:
            s: 全局弧长

        Returns:
            偏航关节曲率
        """
        if s < 0:
            return 0
        k = self.get_kappa(s)
        psi = self.get_psi(s)   # 这里少积分项，要更改
        return k * math.cos(psi)

    #TODO: 积分部分先调包，确认编程思路的正确性，后续再改进，看哪种方式更好
    def get_joint_angle(self, joint_number: int, s_h: float, limit: float, rate: float) -> List[float]:
        """
        计算关节角度（通过积分）

        Args:
            joint_number: 关节数量
            s_h: 蛇头位置

        Returns:
            关节角度（弧度）列表
        """
        joint_angles = []

        for j in range(joint_number):
            if j % 2 == 0:
                angle_rad = rate * integrate.quad(self.get_pitch_curvature, s_h - (j + 1) * l, s_h - (j - 1) * l)[0]
            else:
                angle_rad = rate * integrate.quad(self.get_yaw_curvature, s_h - (j + 1) * l, s_h - (j - 1) * l)[0]

            if angle_rad <= -limit or angle_rad >= limit:
                raise ValueError(f"The value {angle_rad} exceeds the limit.")

            joint_angles.append(angle_rad)

        return joint_angles

    def set_initial_psi(self, psi_0: float):
        """设置初始扭转角"""
        self.psi_0 = psi_0

    def clear(self):
        """清空所有曲线段"""
        self.segments.clear()
        self.psi_array.clear()
        self.accum_lengths.clear()
        self.total_length = 0.0

    def get_total_length(self) -> float:
        """获取总曲线长度"""
        return self.total_length



#TODO:仿真
ser = None                  # 串口
NUM_JOINTS = 40             # 蛇形机器人的关节数量
l = 70                      # 关节长度
# 人机交互参数
step = 1
time_sleep = 0.02           # 连续执行步数之间的睡眠时间
time_sleep_init = 0.02      # 步态初始化时各关节之间的睡眠时间
delta_s = l * 0.5           # 实现shift control时每次递增的增量
m = mujoco.MjModel.from_xml_path('../mujoco_model/Snake_Robot_o40_friction.xml')
d_m = mujoco.MjData(m)

def MuJoCo_Viewer(data_ctrl):
    with viewer.lock():
        d_m.ctrl = data_ctrl
        for i in range(50):
            mujoco.mj_step(m, d_m)
        viewer.sync()
        time.sleep(time_sleep)

#TODO:实机
# ser = SerialPort()          # 串口
# NUM_JOINTS = 20             # 蛇形机器人的关节数量
# l = 70                      # 关节长度
# # 人机交互参数
# step = 1
# time_sleep = 0.5            # 连续执行步数之间的睡眠时间，0.05时行动较为光滑 0.5
# time_sleep_init = 0.5       # 步态初始化时各关节之间的睡眠时间 0.5
# delta_s = l * 0.1           # 实现shift control时每次递增的增量


'''
f5反向[单步调试]，f6正向[单步调试]
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

# 注册热键
keyboard.add_hotkey('f5', reverse_single_step)  # 反向[单步调试]
keyboard.add_hotkey('f6', single_step)          # 正向[单步调试]

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

'''********************************************************以上与具体步态无关********************************************************'''

h = 175  # 履带高
w = 200  # 履带宽
d = 180  # 履带距离

r_c = math.sqrt(h ** 2 + (w / 2) ** 2) / 2  # 圆弧半径
alpha = 2 * math.atan(w / (2 * h))  # 圆弧之间扭转角

l_j = 2 * r_c + d  # 直线长度

# 创建曲线段拼接器
curve = CurveSegments()

# 添加曲线段 - 实现履带步态
print("=== 添加曲线段 ===")
curve.add_curve(LineSegment(l_j), psi=0)  # 直线段
curve.add_curve(ArcSegment(r_c, math.pi), psi=alpha)  # 圆弧段
curve.add_curve(ArcSegment(r_c, math.pi), psi=0)  # 圆弧段
curve.add_curve(LineSegment(l_j), psi=0)  # 直线段
curve.add_curve(ArcSegment(r_c, math.pi), psi=-alpha)  # 圆弧段
curve.add_curve(ArcSegment(r_c, math.pi), psi=0)  # 圆弧段

# TODO：控制变量
s_h = 0.0       # shift control
Psi_0 = 0.0     # rolling control
x = 0           # 本代码中作为翻滚恢复运动的中间量
FLAG = False
index_i = 0

def Gait(time_init = 0.0):
    global step, s_h, Psi_0, x, FLAG, index_i
    while(step > 0):
        index_i += 1
        # 仿真
        data_ctrl = curve.get_joint_angle(NUM_JOINTS, s_h, 1.57, 1.0)
        MuJoCo_Viewer(data_ctrl)
        # 实机
        # data_ctrl = curve.get_joint_angle(NUM_JOINTS, s_h, 90, 57.2958)
        # 发送数据

        time.sleep(time_sleep)

        if index_i > 150:
            # 控制翻滚恢复运动
            new_alpha = (1 - x) * alpha + x * (2 * math.pi - alpha)
            curve.update_segment_psi(1, new_alpha)
            curve.update_segment_psi(4, -new_alpha)
            x += 0.02
            if x >= 1:
                FLAG = True
        elif index_i > 100:
            # 控制Rolling运动
            Psi_0 += 0.1
            curve.set_initial_psi(Psi_0)
        else:
            # 控制前进运动
            s_h += delta_s

        print(index_i)

        step = step - 1
        if step == 0:
            print("\nNow the s_h = ", s_h)
            print("Now the Psi_0 = ", Psi_0)


# TODO：仿真
with mujoco.viewer.launch_passive(m, d_m) as viewer:
    print()
    print("initialize ... ")
    Gait(time_init = time_sleep_init)      # 初始化，将step=1消耗掉
    print("initialize ... Over!")

    while True:
        Gait()
        if FLAG:
            print("snake robot locomote ... Finish!\n")
            break



curve.update_segment_params(1, {"radius": r_c, "angle": 0.1})
print("\n=== 修改曲线参数-改变圆心角 ===")
for info in curve.get_all_segments_info():
    print(f"段{info['index']}: {info['type']}, 长度={info['length']:.2f}, "
          f"扭转角={info.get('psi', 0):.3f}")
print(f"新的总长度: {curve.get_total_length():.2f}")


curve.update_segment_params(1, {"radius": r_c, "angle": 0})
print("\n=== 修改曲线参数-删去长度为0的曲线段 ===")
for info in curve.get_all_segments_info():
    print(f"段{info['index']}: {info['type']}, 长度={info['length']:.2f}, "
          f"扭转角={info.get('psi', 0):.3f}")
print(f"新的总长度: {curve.get_total_length():.2f}")

curve.add_curve(LineSegment(100), 2, psi=0)  # 直线段
print("\n=== 增添曲线段 ===")
for info in curve.get_all_segments_info():
    print(f"段{info['index']}: {info['type']}, 长度={info['length']:.2f}, "
          f"扭转角={info.get('psi', 0):.3f}")
print(f"新的总长度: {curve.get_total_length():.2f}")

# 本代码进程使用有问题，待完善

