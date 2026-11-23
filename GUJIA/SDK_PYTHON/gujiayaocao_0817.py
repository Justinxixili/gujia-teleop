import serial
import serial.tools.list_ports
import time
import math
import configparser
import os
import sys
import threading
import json
from typing import Optional, List, Dict
import signal
import logging
from pynput import keyboard
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS

ROBOT_IP = os.environ.get("GUJIAYAOCAO_ROBOT_IP", "192.168.1.190")
ENCODER_PORT = os.environ.get("GUJIAYAOCAO_ENCODER_PORT", "/dev/ttyACM0")
ENCODER_BAUDRATE = int(os.environ.get("GUJIAYAOCAO_ENCODER_BAUDRATE", "1000000"))

# START_POSE_A = [90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0]
# START_POSE_B = [-90.0, -90.0, 90.0, -90.0, 0.0, 0.0, 0.0]
START_POSE_A = [120.0, -90.0, -90.0, -130.0, 0.0, 20.0, 0.0]
START_POSE_B = [-120.0, -90.0, 90.0, -130.0, 0.0, 20.0, 0.0]
START_POSE_TOL_DEG = 5.0
SKELETON_ALIGN_TOL_DEG = 10.0
SKELETON_REALIGN_TOL_DEG = float(os.environ.get("GUJIAYAOCAO_SKELETON_REALIGN_TOL_DEG", "30.0"))
GATE_STATUS_INTERVAL_SEC = 2.0
TELEOP_START_STEP_DEG = float(os.environ.get("GUJIAYAOCAO_TELEOP_START_STEP_DEG", "0.2"))
HOME_STEP_DEG = float(os.environ.get("GUJIAYAOCAO_HOME_STEP_DEG", "0.2"))
GRIPPER_CMD_HZ = float(os.environ.get("GUJIAYAOCAO_GRIPPER_CMD_HZ", "50"))
GRIPPER_RECV_HZ = float(os.environ.get("GUJIAYAOCAO_GRIPPER_RECV_HZ", "10"))
ALIGN_STAGE_ARM = "arm"
ALIGN_STAGE_SKELETON = "skeleton"
ALIGN_STAGE_READY = "ready"


current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)

from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS
# =====================【新增：导入达妙电机库】=====================
from KM_CAN import (
    Motor, KMGripperControl, KM_Motor_Type, Control_Type, ERROR_MESSAGES
)
# =================================================================
# 踏板当前状态
pedal_state = {
    "left": False,
    "mid": False,
    "right": False
}
# 脚踏按下事件锁存。脚踏键盘的按下/松开脉冲可能很短，主循环轮询会漏读。
pedal_press_latch = {
    "left": False,
    "mid": False,
    "right": False
}
pedal_lock = threading.Lock()

# 【新增】回home忙标记，防止按住踏板反复执行回home
home_running = False
teleop_interp_a = None
teleop_interp_b = None

# 脚踏按键映射：左=s，右=d
KEY_LEFT  = keyboard.KeyCode(char='s')
KEY_MID   = keyboard.KeyCode(char='q')
KEY_RIGHT = keyboard.KeyCode(char='d')


def get_pedal_name(key):
    """把 pynput 的按键对象转换为脚踏名称，兼容大小写字母。"""
    char = getattr(key, "char", None)
    if not char:
        return None

    char = char.lower()
    if char == "s":
        return "left"
    if char == "q":
        return "mid"
    if char == "d":
        return "right"
    return None


def consume_pedal_press(name):
    """消费一次脚踏按下事件，避免短脉冲被主循环漏掉。"""
    with pedal_lock:
        pressed = pedal_press_latch[name]
        pedal_press_latch[name] = False
        return pressed


def max_abs_joint_error(actual, target):
    return max(abs(actual[i] - target[i]) for i in range(7))


def format_joint_list(joints):
    return "[" + ", ".join(f"{joint:.1f}" for joint in joints) + "]"


def step_joint_pose(current, target, max_step_deg):
    next_pose = []
    done = True
    for cur, dst in zip(current, target):
        delta = dst - cur
        if abs(delta) > max_step_deg:
            cur += math.copysign(max_step_deg, delta)
            done = False
        else:
            cur = dst
        next_pose.append(cur)
    return next_pose, done


def emit_alignment_status(
    a_real,
    b_real,
    target_a,
    target_b,
    filter_ok_a,
    filter_ok_b,
    going,
    stage,
    skeleton_aligned_a=None,
    skeleton_aligned_b=None,
):
    start_err_a = [abs(a_real[i] - START_POSE_A[i]) for i in range(7)]
    start_err_b = [abs(b_real[i] - START_POSE_B[i]) for i in range(7)]
    check_skeleton = stage in (ALIGN_STAGE_SKELETON, ALIGN_STAGE_READY)
    skeleton_err_a = [abs(a_real[i] - target_a[i]) for i in range(7)] if check_skeleton and filter_ok_a else [None] * 7
    skeleton_err_b = [abs(b_real[i] - target_b[i]) for i in range(7)] if check_skeleton and filter_ok_b else [None] * 7
    payload = {
        "start_tol": START_POSE_TOL_DEG,
        "skeleton_tol": SKELETON_ALIGN_TOL_DEG,
        "skeleton_realign_tol": SKELETON_REALIGN_TOL_DEG,
        "going": going,
        "stage": stage,
        "A": {
            "start_err": start_err_a,
            "skeleton_err": skeleton_err_a,
            "skeleton_aligned": skeleton_aligned_a or [False] * 7,
        },
        "B": {
            "start_err": start_err_b,
            "skeleton_err": skeleton_err_b,
            "skeleton_aligned": skeleton_aligned_b or [False] * 7,
        },
    }
    print("__ALIGN_STATUS__" + json.dumps(payload, ensure_ascii=False))


def on_key_press(key):
    """键盘回调：只更新踏板状态，不做机器人操作"""
    global running
    pedal_name = get_pedal_name(key)
    with pedal_lock:
        if pedal_name is not None and not pedal_state[pedal_name]:
            pedal_press_latch[pedal_name] = True
        if pedal_name is not None:
            pedal_state[pedal_name] = True

        if pedal_name == "left":
            print("[KEY] 收到 s 左脚踏按下")
        if pedal_name == "right":
            print("[KEY] 收到 d 右脚踏按下")

    # ESC按下，结束整个程序
    if key == keyboard.Key.esc:
        running = False
        return False


def on_key_release(key):
    """松开踏板，更新状态"""
    pedal_name = get_pedal_name(key)
    with pedal_lock:
        if pedal_name is not None:
            pedal_state[pedal_name] = False

        if pedal_name == "left":
            print("[KEY] s 左脚踏松开")
        if pedal_name == "right":
            print("[KEY] d 右脚踏松开")

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)
# logger.setLevel(logging.DEBUG)

'''初始化订阅数据的结构体'''
dcss=DCSS()

'''初始化机器人接口'''
robot=Marvin_Robot()

'''查验连接是否成功'''
init = robot.connect(ROBOT_IP)
if init==0:
    logger.error('failed:端口占用，连接失败!')
    exit(0)
else:
    '''防总线通信异常,先清错'''
    time.sleep(0.5)
    robot.clear_set()
    robot.clear_error('A')
    robot.clear_error('B')
    robot.send_cmd()
    time.sleep(0.5)

    motion_tag = 0
    frame_update = None
    for i in range(10):
        sub_data = robot.subscribe(dcss)
        print(f"connect frames :{sub_data['outputs'][0]['frame_serial']}")
        if sub_data['outputs'][0]['frame_serial'] != 0 and frame_update != sub_data['outputs'][0]['frame_serial']:
            motion_tag += 1
            frame_update = sub_data['outputs'][0]['frame_serial']
        time.sleep(0.1)
    if motion_tag > 0:
        logger.info('success:机器人连接成功!')
    else:
        logger.error('failed:机器人连接失败!')
        exit(0)


'''开启日志以便检查'''
robot.log_switch('1')
robot.local_log_switch('1')
time.sleep(1)

'''清错'''
robot.clear_set()
robot.clear_error('A')
robot.clear_error('B')
robot.send_cmd()
time.sleep(1)


# =====================【正确设置工具负载，阻抗模式之前执行】======================
# --------请根据你的实际夹具修改下面参数--------
# kineParams [X,Y,Z mm, A,B,C deg] 法兰到工具末端偏移
tool_kine_A = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
tool_dyn_A = [0.5,0.0, 0.0, 50.0,0.004, 0.000, 0.00,0.004, 0.000, 0.03]

tool_kine_B = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
tool_dyn_B = [0.5,0.0, 0.0, 50.0,0.004, 0.000, 0.00,0.004, 0.000, 0.03]
robot.clear_set()
ret_a = robot.set_tool("A", kineParams=tool_kine_A, dynamicParams=tool_dyn_A)
ret_b = robot.set_tool("B", kineParams=tool_kine_B, dynamicParams=tool_dyn_B)
robot.send_cmd()
time.sleep(0.5)
logger.info(f"设置工具 A={ret_a}, B={ret_b} | 1=成功,2=失败")
if ret_a != 1 or ret_b !=1:
    logger.warning("警告：工具设置返回非1，阻抗动力学补偿可能失效！")
# =========================================================================
time.sleep(0.5)

logger.info("已设置A/B臂工具负载，准备进入关节阻抗扭矩模式")

'''阻抗参数'''
robot.clear_set()
robot.set_joint_kd_params(arm='A',K=[12,12,12,10, 9, 9, 7], D=[0.1,0.1,0.1,0.1,0.1,0.1,0.1])
robot.set_joint_kd_params(arm='B',K=[12,12,12,10, 9, 9, 7], D=[0.1,0.1,0.1,0.1,0.1,0.1,0.1])
robot.send_cmd()
time.sleep(1)

'''开启A/B双臂速度前馈'''
ctrl_hz = 100
vel_step_ms = max(1, int(round(1000.0 / ctrl_hz)))
robot.clear_set()
ok_a = robot.set_vel_est_step(arm='A', time=vel_step_ms)
ok_b = robot.set_vel_est_step(arm='B', time=vel_step_ms)
robot.send_cmd()
logger.info(f"速度前馈周期{vel_step_ms}ms，A臂开启{ok_a}, B臂开启{ok_b}")
time.sleep(0.2)
# 全局退出标志
running = True

'''设置扭矩模式,关节阻抗模式,速度加速度百分比'''
robot.clear_set()
robot.set_state(arm='A',state=3)
robot.set_state(arm='B',state=3)
robot.set_impedance_type(arm='A',type=1)
robot.set_impedance_type(arm='B',type=1)
robot.set_vel_acc(arm='A',velRatio=100, AccRatio=100)
robot.set_vel_acc(arm='B',velRatio=100, AccRatio=100)
robot.send_cmd()
time.sleep(1)

'''设置拖动模式和参数'''
robot.clear_set()
robot.set_drag_space(arm='A',dgType=1)
robot.set_drag_space(arm='B',dgType=1)
robot.send_cmd()
logger.info("已进入关节拖动模式。请手动拖动到：L/A=[90,-90,-90,-90,0,0,0]，R/B=[-90,-90,90,-90,0,0,0]，误差需在±2°内。")
time.sleep(0.5)


def set_joint_drag_mode(enable: bool):
    dg_type = 1 if enable else 0
    robot.clear_set()
    robot.set_drag_space(arm='A',dgType=dg_type)
    robot.set_drag_space(arm='B',dgType=dg_type)
    robot.send_cmd()
    print(f"[DRAG] joint drag {'enabled' if enable else 'disabled'}")


def signal_handler(sig, frame):
    global running
    print("\n\n正在退出程序...")
    time.sleep(0.5)
    running = False

signal.signal(signal.SIGINT, signal_handler)


class STSDriverConfig:
    timeout_ms: int = 500
    baudrate_map = {
        9600: 9600, 19200: 19200, 38400: 38400, 57600: 57600,
        115200: 115200, 230400: 230400, 460800: 460800, 500000: 500000,
        576000: 576000, 921600: 921600, 1000000: 1000000
    }

class STSDriver:
    """STS编码器串口驱动【恢复原版驱动类】"""
    def __init__(self, port: str, baudrate: int):
        self.fd: Optional[serial.Serial] = None
        self.timeout_ms: int = STSDriverConfig.timeout_ms
        self.ser_mutex = threading.Lock()

        try:
            baud = STSDriverConfig.baudrate_map.get(baudrate, 115200)
            self.fd = serial.Serial(
                port=port, baudrate=baud, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                xonxoff=False, rtscts=False, dsrdtr=False,
                timeout=self.timeout_ms / 1000.0
            )
            if self.fd.is_open:
                print(f"[串口] 已打开: {port} @ {baudrate} 波特率")
            else:
                raise RuntimeError("串口打开失败")
        except Exception as e:
            raise RuntimeError(f"打开串口失败: {str(e)}")

    def __del__(self):
        if self.fd and self.fd.is_open:
            self.fd.close()
            print("[串口] 已关闭")

    def calculate_checksum(self, data: List[int]) -> int:
        sum_val = sum(data)
        return sum_val & 0xFF

    def send_and_receive(self, cmd: List[int]) -> Optional[List[int]]:
        if not self.fd or not self.fd.is_open:
            return None

        with self.ser_mutex:
            self.fd.reset_input_buffer()
            try:
                self.fd.write(bytes(cmd))
                time.sleep(0.001)
            except Exception as e:
                print(f"发送数据失败: {e}")
                return None

            response = []
            start_time = time.time()
            no_data_count = 0

            while True:
                elapsed = (time.time() - start_time) * 1000
                if elapsed > self.timeout_ms:
                    break

                if self.fd.in_waiting > 0:
                    data = self.fd.read(self.fd.in_waiting)
                    response.extend(list(data))
                    no_data_count = 0
                else:
                    no_data_count += 1
                    time.sleep(0.001)
                    if no_data_count >= 2 and response:
                        break
        return response if response else None


class EncoderReader:
    """编码器核心处理类，增加一阶低通滤波 + 夹爪遥操作"""
    def __init__(self):
        # ===================== 可手动修改配置 =====================
        self.port = ENCODER_PORT
        self.baudrate = ENCODER_BAUDRATE
        self.send_frequency = 100.0
        self.filter_id = 0
        self.enable_checksum = True
        self.encoder_resolution = 4096
      
        # 获取当前py脚本所在文件夹
        script_path = os.path.abspath(__file__)
        script_dir = os.path.dirname(script_path)
        # 拼接得到ini完整绝对路径，固定地址
        self.config_file = os.path.join(script_dir, "servo_config.ini")

        # 校验配置文件是否存在，不存在抛出异常终止程序，打印完整路径方便排查
        if not os.path.isfile(self.config_file):
            raise FileNotFoundError(
                f"伺服配置文件缺失！\n预期路径：{self.config_file}\n请确认servo_config.ini放置到脚本同级目录"
            )
        # 一阶低通滤波系数 α 0~1；越小滤波越强，响应越慢
        self.filter_alpha = 0.9
        
        # ----------------【夹爪硬件配置，请修改为你的实际硬件】----------------
        # STS编码器ID用于控制夹爪
        self.enc_left_gripper_id  = 7
        self.enc_right_gripper_id = 15
        # STS编码器摆动角度范围(deg)
        self.enc_min_deg = 0.0
        self.enc_max_deg = 90.0
        # 达妙DM4310夹爪电机CAN ID
        self.motor_left_slave  = 1
        self.motor_right_slave = 2
        self.motor_left_master  = self.motor_left_slave + 0x10
        self.motor_right_master = self.motor_right_slave + 0x10
        # MIT控制参数
        self.grip_kp = 3.0
        self.grip_kd = 0.12
        self.grip_tau_ff = 0.0
        # 夹爪电机机械限位 rad
        self.gripper_qmin = 0.0
        self.gripper_qmax = 1.3
        self.gripper_teleop_enable = True   # 是否开启夹爪跟随
        self.gripper_cmd_period = 1.0 / max(1.0, GRIPPER_CMD_HZ)
        self.gripper_recv_period = 1.0 / max(1.0, GRIPPER_RECV_HZ)
        self.next_gripper_cmd_time = 0.0
        self.next_gripper_recv_time = 0.0
        # ======================================================================

        self.radian_scale = 2 * math.pi / self.encoder_resolution
        self.cmd = [0xFF,0xFF,0xFE,0x14,0x82,0x38,0x02,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0xA9]

        self.last_raw_position: Dict[int, int] = {}
        self.accumulated_position: Dict[int, int] = {}
        self.zero_offset_raw: Dict[int, int] = {}
        self.direction: Dict[int, int] = {}
        self.data_mutex = threading.Lock()

        self.angle = [0.0]*16          # 原始角度(度)
        self.filtered_angle = [0.0]*16 # 一阶低通滤波输出角度(度)
        self.filter_init_flag = [False]*16 #滤波初始化标记

        #初始化串口驱动
        self.driver = STSDriver(self.port, self.baudrate)
        self.load_config()

        # =====================【初始化夹爪控制器】=====================
        self.gripper_ctrl = KMGripperControl(robot=robot)
        self.motor_left_grip  = Motor(KM_Motor_Type.DM4310, self.motor_left_slave,  self.motor_left_master)
        self.motor_right_grip = Motor(KM_Motor_Type.DM4310, self.motor_right_slave, self.motor_right_master)
        self.gripper_ctrl.addMotor(self.motor_left_grip)
        self.gripper_ctrl.add_to_ch(self.motor_left_grip, "left")   # left → A网关485
        self.gripper_ctrl.addMotor(self.motor_right_grip)
        self.gripper_ctrl.add_to_ch(self.motor_right_grip, "right") # right → B网关485
        # ============================================================

        print("="*65)
        print("命令说明：")
        print("  set_zero 1    - 设置编码器1零点")
        print("  set_dir 1 -1  - 设置编码器1方向")
        print("  freq           - 查看当前下发频率")
        print("  set_filter 0.2 - 设置一阶低通滤波系数 0~1")
        print("  gripper_enable   - 使能夹爪电机")
        print("  gripper_disable  - 失能夹爪电机")
        print("  gripper_toggle   - 开关夹爪编码器遥跟随")
        print("  gripper_status   - 打印夹爪位置/力矩/温度/故障")
        print("  quit          - 退出程序")
        print("="*65)

    # =====================【新增夹爪工具函数】=====================
    def map_encoder_to_q(self, enc_deg:float)->float:
        """编码器角度(deg)映射到电机目标位置rad，双重软限位"""
        enc_clamp = max(self.enc_min_deg, min(self.enc_max_deg, enc_deg))
        norm = (enc_clamp - self.enc_min_deg)/(self.enc_max_deg - self.enc_min_deg)
        q_target = norm * (self.gripper_qmax - self.gripper_qmin) + self.gripper_qmin
        return max(self.gripper_qmin, min(self.gripper_qmax, q_target))

    def gripper_enable_all(self):
        print("[夹爪]使能左夹爪电机")
        self.gripper_ctrl.enable(self.motor_left_grip)
        print("[夹爪]使能右夹爪电机")
        self.gripper_ctrl.enable(self.motor_right_grip)

    def gripper_disable_all(self):
        print("[夹爪]失能左右夹爪电机")
        self.gripper_ctrl.disable(self.motor_left_grip)
        self.gripper_ctrl.disable(self.motor_right_grip)

    def print_gripper_status(self):
        print("\n----------【左夹爪电机】----------")
        q = self.motor_left_grip.getPosition()
        dq = self.motor_left_grip.getVelocity()
        tau = self.motor_left_grip.getTorque()
        err = self.motor_left_grip.getErr()
        t_mos = self.motor_left_grip.getTempMos()
        t_motor = self.motor_left_grip.getTempMotor()
        print(f"q={q:.4f} rad | dq={dq:.3f} rad/s | τ={tau:.3f} Nm")
        print(f"故障码:{err} {ERROR_MESSAGES.get(err,'未知故障')} | MOS:{t_mos}℃ 电机:{t_motor}℃")

        print("\n----------【右夹爪电机】----------")
        qr = self.motor_right_grip.getPosition()
        dqr = self.motor_right_grip.getVelocity()
        taur = self.motor_right_grip.getTorque()
        errr = self.motor_right_grip.getErr()
        t_mos_r = self.motor_right_grip.getTempMos()
        t_motor_r = self.motor_right_grip.getTempMotor()
        print(f"q={qr:.4f} rad | dq={dqr:.3f} rad/s | τ={taur:.3f} Nm")
        print(f"故障码:{errr} {ERROR_MESSAGES.get(errr,'未知故障')} | MOS:{t_mos_r}℃ 电机:{t_motor_r}℃\n")
    # =============================================================

    def get_robot_real_joint(self):
        global dcss
        armA_data = dcss.m_Out[0]
        armB_data = dcss.m_Out[1]
        a_real = [armA_data.m_FB_Joint_Pos[i] for i in range(7)]
        b_real = [armB_data.m_FB_Joint_Pos[i] for i in range(7)]
        return a_real, b_real

    def get_send_frequency(self):
        return self.send_frequency

    def load_config(self):
        if not os.path.exists(self.config_file):
            print("[配置] 无配置文件，使用默认值")
            return
        config = configparser.ConfigParser()
        config.read(self.config_file, encoding="utf-8")
        for sec in config.sections():
            try:
                sid = int(sec)
                self.zero_offset_raw[sid] = int(config[sec]["zero_offset_raw"])
                self.direction[sid] = int(config[sec]["direction"])
            except:
                continue
        print(f"[配置] 已加载: {self.config_file}")

    def save_config(self):
        config = configparser.ConfigParser()
        all_ids = set(self.zero_offset_raw.keys()) | set(self.direction.keys())
        for sid in all_ids:
            config[str(sid)] = {
                "zero_offset_raw": str(self.zero_offset_raw.get(sid, 0)),
                "direction": str(self.direction.get(sid, 1))
            }
        with open(self.config_file, "w", encoding="utf-8") as f:
            f.write("# STS编码器配置\n# zero_offset_raw=零点原始值\n# direction=1正向/-1反向\n\n")
            config.write(f)
        print(f"[配置] 已保存: {self.config_file}")

    def verify_checksum(self, data: List[int], offset: int) -> bool:
        if offset +7 >= len(data): return False
        s = sum(data[offset+2:offset+7]) & 0xFF
        return (~s & 0xFF) == data[offset+7]

    def get_accumulated_position(self, sid: int, raw: int) -> int:
        with self.data_mutex:
            if sid not in self.last_raw_position:
                self.accumulated_position[sid] = raw - self.zero_offset_raw.get(sid, 0)
                self.last_raw_position[sid] = raw
                return self.accumulated_position[sid]

            last = self.last_raw_position[sid]
            delta = raw - last
            if delta > self.encoder_resolution//2: delta -= self.encoder_resolution
            if delta < -self.encoder_resolution//2: delta += self.encoder_resolution

            self.accumulated_position[sid] += delta
            self.last_raw_position[sid] = raw
            return self.accumulated_position[sid]

    def get_radian(self, sid: int, raw: int) -> float:
        acc = self.get_accumulated_position(sid, raw)
        dir_val = self.direction.get(sid, 1)
        angle = math.atan2(math.sin(acc * dir_val * self.radian_scale), math.cos(acc * dir_val * self.radian_scale))
        return angle

    def set_zero(self, sid: int) -> bool:
        with self.data_mutex:
            if sid not in self.last_raw_position:
                print(f"错误：未找到编码器 {sid}")
                return False
            raw = self.last_raw_position[sid]
            self.zero_offset_raw[sid] = raw
            self.accumulated_position[sid] = 0
            self.save_config()
            print(f"成功：编码器 {sid} 零点已重置")
            return True

    def set_direction(self, sid: int, new_dir: int) -> bool:
        if new_dir not in (1, -1):
            print("错误：方向只能是 1 或 -1")
            return False
        with self.data_mutex:
            if sid not in self.last_raw_position:
                print(f"错误：未找到编码器 {sid}")
                return False
            self.direction[sid] = new_dir
            self.save_config()
            print(f"成功：编码器 {sid} 方向已设为 {new_dir}")
            return True

    def parse_data(self, data: List[int]):
        """解析编码器 + 一阶低通滤波"""
        if len(data) != 128: return
        GROUP = 8
        valid = True
        for i in range(len(data)//GROUP):
            off = i*GROUP
            if data[off]!=0xFF or data[off+1]!=0xFF:
                valid=False
                break
            if self.enable_checksum and not self.verify_checksum(data, off):
                valid=False
                break
        if not valid: return

        for i in range(len(data)//GROUP):
            off = i*GROUP
            sid = data[off+2]
            raw = (data[off+6] <<8) | data[off+5]
            rad = self.get_radian(sid, raw)
            deg = rad * 180 / math.pi
            self.angle[i] = deg
           
            # ========= 一阶低通滤波 y = α*x + (1-α)*y_prev =========
            if not self.filter_init_flag[i]:
                self.filtered_angle[i] = deg
                self.filter_init_flag[i] = True
            else:
                self.filtered_angle[i] = self.filter_alpha * deg + (1.0 - self.filter_alpha) * self.filtered_angle[i]

            if self.filter_id !=0 and sid != self.filter_id:
                continue       
    def read_loop(self):
        global running, dcss, going, teleop_interp_a, teleop_interp_b
        while running:
            period = 1.0 / self.get_send_frequency()
            start = time.time()
            res = self.driver.send_and_receive(self.cmd)
            if res:
                # ✅无论遥操作开不开，永远解析编码器，刷新filtered_angle
                self.parse_data(res)

                # =========只有遥操作开启 going=False，才下发从手机械臂指令=========
                if not going:
                    robot.clear_set()
                    # 使用【滤波后】角度下发机械臂本体
                    joint_cmd_1=[
                        self.filtered_angle[0],self.filtered_angle[1],self.filtered_angle[2],
                        self.filtered_angle[3],self.filtered_angle[4],self.filtered_angle[5],
                        self.filtered_angle[6]
                    ]
                    joint_cmd_2=[
                        self.filtered_angle[8],self.filtered_angle[9],self.filtered_angle[10],
                        self.filtered_angle[11],self.filtered_angle[12],self.filtered_angle[13],
                        self.filtered_angle[14]
                    ]
                    if teleop_interp_a is not None and teleop_interp_b is not None:
                        teleop_interp_a, done_a = step_joint_pose(
                            teleop_interp_a,
                            joint_cmd_1,
                            TELEOP_START_STEP_DEG,
                        )
                        teleop_interp_b, done_b = step_joint_pose(
                            teleop_interp_b,
                            joint_cmd_2,
                            TELEOP_START_STEP_DEG,
                        )
                        joint_cmd_1 = list(teleop_interp_a)
                        joint_cmd_2 = list(teleop_interp_b)
                        if done_a and done_b:
                            teleop_interp_a = None
                            teleop_interp_b = None
                    robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
                    robot.set_joint_cmd_pose(arm='B',joints=joint_cmd_2)
                    robot.send_cmd()
                    # =====================【夹爪遥操作逻辑】=====================
                    now = time.monotonic()
                    if self.gripper_teleop_enable and now >= self.next_gripper_cmd_time:
                        self.next_gripper_cmd_time = now + self.gripper_cmd_period
                        q_l_norm = abs(self.filtered_angle[7]) / 9.0
                        q_r_norm = abs(self.filtered_angle[15]) /9.0
                        q_l_norm =max(0.0,min(1.0,q_l_norm))
                        q_r_norm =max(0.0,min(1.0,q_r_norm))
                        q_l_target = -0.15 +q_l_norm *(1.1-(-0.15))
                        q_r_target = -0.15 +q_r_norm *(1.1-(-0.15))
                        # MIT模式下发
                        self.gripper_ctrl.controlMIT(
                            self.motor_left_grip, self.grip_kp, self.grip_kd,
                            q_l_target, 0.0, self.grip_tau_ff
                        )
                        self.gripper_ctrl.controlMIT(
                            self.motor_right_grip, self.grip_kp, self.grip_kd,
                            q_r_target, 0.0, self.grip_tau_ff
                        )
                    if now >= self.next_gripper_recv_time:
                        self.next_gripper_recv_time = now + self.gripper_recv_period
                        self.gripper_ctrl.recv()   #夹爪反馈刷新

                # ❗robot.subscribe(dcss)移到主线程，不要放在编码器线程里面
            # 周期控频，保证循环持续跑
            elapsed = time.time() - start
            sleep_time = max(0, period - elapsed)
            time.sleep(sleep_time)

    def cmd_loop(self):
        global running
        while running:
            time.sleep(0.2)
            
def main():
    global running, going, home_running, teleop_interp_a, teleop_interp_b
    reader = None
    listener = None
    try:
        going = True
        home_running = False
        teleop_interp_a = None
        teleop_interp_b = None
        reader = EncoderReader()
        reader.gripper_enable_all()
        t1 = threading.Thread(target=reader.read_loop, daemon=True)
        t2 = threading.Thread(target=reader.cmd_loop, daemon=True)
        t1.start()
        t2.start()

        # 后台启动脚踏键盘监听
        listener = keyboard.Listener(on_press=on_key_press, on_release=on_key_release)
        listener.daemon = True
        listener.start()
        print("✅脚踏监听启动：s左踏板按下=回Home；d右踏板按下瞬间=开启摇操，ESC退出")
        print(f"[GATE] 第一步：先对齐机械臂到 L/A={format_joint_list(START_POSE_A)}，R/B={format_joint_list(START_POSE_B)}，允许误差±{START_POSE_TOL_DEG:.1f}°。")
        print(f"[GATE] 第二步：机械臂对齐后，再对齐骨架编码器；每个骨架关节只需进入一次≤{SKELETON_ALIGN_TOL_DEG:.1f}°误差范围。")
        last_gate_status_time = 0.0
        skeleton_aligned_a = [False] * 7
        skeleton_aligned_b = [False] * 7
        teleop_has_started = False
        skeleton_realign_required = False
        # =========主程序循环========================

        while running:
            sub_data = robot.subscribe(dcss)
            time.sleep(0.2)
            a_real, b_real = reader.get_robot_real_joint()
            # 消费脚踏按下事件。事件由键盘回调锁存，短脉冲不会被主循环漏掉。
            left_pressed = consume_pedal_press("left")
            right_pressed = consume_pedal_press("right")

            # =====================【持续实时对比：机械臂真实角度 VS 编码器角度】=====================
            MAX_ALLOW_ERR_DEG = SKELETON_ALIGN_TOL_DEG   # 最大允许单关节角度误差(度)

            filter_ok_a = False
            filter_ok_b = False
            max_err_a = 999.0
            max_err_b = 999.0
            with reader.data_mutex:
                target_A = reader.filtered_angle[0:7]   # A臂7轴目标
                target_B = reader.filtered_angle[8:15]  # B臂7轴目标
                filter_ok_a = all(reader.filter_init_flag[i] for i in range(7))
                filter_ok_b = all(reader.filter_init_flag[i] for i in range(8,15))

            start_err_a = max_abs_joint_error(a_real, START_POSE_A)
            start_err_b = max_abs_joint_error(b_real, START_POSE_B)
            start_pose_ok = start_err_a <= START_POSE_TOL_DEG and start_err_b <= START_POSE_TOL_DEG
            if not start_pose_ok and not teleop_has_started and not skeleton_realign_required:
                skeleton_aligned_a = [False] * 7
                skeleton_aligned_b = [False] * 7

            # 只有初始化完成才计算误差，否则误差置很大
            if filter_ok_a and filter_ok_b:
                skeleton_err_a = [abs(a_real[i] - target_A[i]) for i in range(7)]
                skeleton_err_b = [abs(b_real[i] - target_B[i]) for i in range(7)]
                max_err_a = max(skeleton_err_a)
                max_err_b = max(skeleton_err_b)
                if start_pose_ok or teleop_has_started or skeleton_realign_required:
                    for i in range(7):
                        if skeleton_err_a[i] <= MAX_ALLOW_ERR_DEG:
                            skeleton_aligned_a[i] = True
                        if skeleton_err_b[i] <= MAX_ALLOW_ERR_DEG:
                            skeleton_aligned_b[i] = True
            max_err = max(max_err_a, max_err_b)

            if (
                going
                and filter_ok_a
                and filter_ok_b
                and all(skeleton_aligned_a)
                and all(skeleton_aligned_b)
                and max_err > SKELETON_REALIGN_TOL_DEG
            ):
                exceeded_a = [err > SKELETON_REALIGN_TOL_DEG for err in skeleton_err_a]
                exceeded_b = [err > SKELETON_REALIGN_TOL_DEG for err in skeleton_err_b]
                for i in range(7):
                    if exceeded_a[i]:
                        skeleton_aligned_a[i] = False
                    if exceeded_b[i]:
                        skeleton_aligned_b[i] = False

                if max_err_a >= max_err_b:
                    exceed_joint = f"A{max(range(7), key=lambda i: skeleton_err_a[i]) + 1}"
                else:
                    exceed_joint = f"B{max(range(7), key=lambda i: skeleton_err_b[i]) + 1}"

                skeleton_realign_required = True
                print(
                    f"[GATE] 骨架点位偏差超过 {SKELETON_REALIGN_TOL_DEG:.1f}°，"
                    f"需要重新对齐超限关节；最大偏差 {max_err:.2f}°({exceed_joint})。"
                )
                last_gate_status_time = 0.0

            skeleton_pose_ok = (
                filter_ok_a
                and filter_ok_b
                and all(skeleton_aligned_a)
                and all(skeleton_aligned_b)
            )
            start_guard_ok = filter_ok_a and filter_ok_b and max_err <= SKELETON_REALIGN_TOL_DEG
            can_start_teleop = (
                skeleton_pose_ok
                and start_guard_ok
                and (start_pose_ok or teleop_has_started or skeleton_realign_required)
            )
            if skeleton_pose_ok and start_guard_ok and (start_pose_ok or teleop_has_started or skeleton_realign_required):
                alignment_stage = ALIGN_STAGE_READY
            elif not start_pose_ok and not teleop_has_started and not skeleton_realign_required:
                alignment_stage = ALIGN_STAGE_ARM
            elif not skeleton_pose_ok:
                alignment_stage = ALIGN_STAGE_SKELETON
            else:
                alignment_stage = ALIGN_STAGE_READY

            now = time.time()
            if going and now - last_gate_status_time >= GATE_STATUS_INTERVAL_SEC:
                emit_alignment_status(
                    a_real,
                    b_real,
                    target_A,
                    target_B,
                    filter_ok_a,
                    filter_ok_b,
                    going,
                    alignment_stage,
                    skeleton_aligned_a,
                    skeleton_aligned_b,
                )
                if alignment_stage == ALIGN_STAGE_ARM:
                    print(
                        f"[GATE] 当前阶段=手臂对齐；起始点误差 A/L={start_err_a:.2f}°, "
                        f"B/R={start_err_b:.2f}°, 阈值={START_POSE_TOL_DEG:.1f}°"
                    )
                elif alignment_stage == ALIGN_STAGE_SKELETON:
                    print(
                        f"[GATE] 当前阶段=骨架对齐；骨架已对齐 A={sum(skeleton_aligned_a)}/7, "
                        f"B={sum(skeleton_aligned_b)}/7；当前最大误差 A={max_err_a:.2f}°, "
                        f"B={max_err_b:.2f}°, 阈值={SKELETON_ALIGN_TOL_DEG:.1f}°"
                    )
                else:
                    print("[GATE] 手臂和骨架均已对齐，可以按右踏板开启摇操。")
                last_gate_status_time = now

            # 【可选】打开实时偏差打印，调试用；正式运行可以注释掉
            #print(f"实时偏差 | A臂={max_err_a:.2f}°, B臂={max_err_b:.2f}°,最大={max_err:.2f}°")
            # ====================================================================================
            # =========左踏板s：按下执行回Home=========
            if left_pressed is True and home_running is False:
                going = True
                teleop_interp_a = None
                teleop_interp_b = None
                home_running = True
                print("【左踏板按下】执行回Home点位")
                robot.clear_set()
                robot.clear_error('A')
                robot.clear_error('B')
                robot.send_cmd()
                time.sleep(1)
                robot.set_state(arm='A',state=1)
                robot.set_state(arm='B',state=1)
                robot.send_cmd()
                time.sleep(1)

                joint_cmd_1 = list(a_real)
                joint_cmd_2 = list(b_real)
                home_period = 1.0 / reader.get_send_frequency()
                max_home_delta = max(
                    max_abs_joint_error(joint_cmd_1, START_POSE_A),
                    max_abs_joint_error(joint_cmd_2, START_POSE_B),
                )
                home_interp_timeout = max(10.0, max_home_delta / max(HOME_STEP_DEG, 0.001) * home_period + 5.0)
                home_interp_start = time.time()
                print(f"[INTERP] 回Home插点过渡，每周期最大步长 {HOME_STEP_DEG:.2f}°")

                while running:
                    joint_cmd_1, done_a = step_joint_pose(
                        joint_cmd_1,
                        START_POSE_A,
                        HOME_STEP_DEG,
                    )
                    joint_cmd_2, done_b = step_joint_pose(
                        joint_cmd_2,
                        START_POSE_B,
                        HOME_STEP_DEG,
                    )

                    robot.clear_set()
                    robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
                    robot.set_joint_cmd_pose(arm='B',joints=joint_cmd_2)
                    robot.send_cmd()

                    if done_a and done_b:
                        break
                    if (time.time() - home_interp_start) > home_interp_timeout:
                        print(f"⚠️回Home插点超时({home_interp_timeout:.1f}s)，停止继续插点")
                        break
                    time.sleep(home_period)

                # 等待双臂运动到位，增加超时保护
                MAX_WAIT_SEC = 10.0
                start_wait = time.time()
                sub_data = robot.subscribe(dcss)

                while True:
                    flag_a = sub_data['outputs'][0]['low_speed_flag']
                    flag_b = sub_data['outputs'][1]['low_speed_flag']
                    print(f"A臂low_speed_flag={flag_a}, B臂low_speed_flag={flag_b}")

                    if flag_a == b'\x01' and flag_b == b'\x01':
                        print("回home点完成！！")
                        break
                    if (time.time() - start_wait) > MAX_WAIT_SEC:
                        print(f"⚠️回Home等待超时({MAX_WAIT_SEC}s)！机械臂未收到到位信号，请检查机械臂状态！")
                        break
                    time.sleep(1.5)
                    sub_data = robot.subscribe(dcss)
                home_running = False
                teleop_has_started = False
                skeleton_realign_required = False
                skeleton_aligned_a = [False] * 7
                skeleton_aligned_b = [False] * 7
                continue
                # 右踏板 d：边沿触发，按下瞬间开启摇操遥操作
            if right_pressed is True:
                if not going:
                    going = True
                    teleop_interp_a = None
                    teleop_interp_b = None
                    robot.clear_set()
                    robot.send_cmd()
                    set_joint_drag_mode(True)
                    print("【右踏板触发】停止摇操下发，进入待机；可踩左踏板回Home，或踩右踏板再次开启摇操。")
                    continue

                if not can_start_teleop and not teleop_has_started and not start_pose_ok:
                    print(
                        f"❌摇操拒绝：第一步未完成，请先手动拖动机械臂到 L/A={format_joint_list(START_POSE_A)}，"
                        f"R/B={format_joint_list(START_POSE_B)}，并保证两臂都在±{START_POSE_TOL_DEG:.1f}°内。"
                    )
                    continue

                if not can_start_teleop and not (filter_ok_a and filter_ok_b):
                    print("⚠️遥操作拒绝：机械臂已对齐，第二步等待骨架编码器滤波初始化完成。")
                    continue

                if not can_start_teleop:
                    print(
                        f"❌遥操作拒绝：机械臂已对齐，第二步请逐个对齐骨架关节。当前已对齐 "
                        f"A={sum(skeleton_aligned_a)}/7, B={sum(skeleton_aligned_b)}/7；"
                        f"当前骨架最大偏差 {max_err:.2f}°，阈值{MAX_ALLOW_ERR_DEG}°。"
                    )
                    continue

                print("【右踏板触发】开启摇操遥操作！")
                # ✅全部条件满足，才开启遥
                teleop_interp_a = list(a_real)
                teleop_interp_b = list(b_real)
                print(f"[INTERP] 摇操启动插点过渡，每周期最大步长 {TELEOP_START_STEP_DEG:.2f}°")
                set_joint_drag_mode(False)
                going = False
                teleop_has_started = True
                skeleton_realign_required = False

            time.sleep(0.1)  # 主循环间隔，降低CPU占用

        # ----------------ESC按下后执行资源清理（只保留一份）----------------
        if reader:
            time.sleep(0.3)
            robot.clear_set()
            robot.set_state(arm='A', state=0)
            robot.set_state(arm='B', state=0)
            robot.send_cmd()
            time.sleep(0.2)
            print("程序正常退出！！")
    except Exception as e:
        print(f"程序启动失败: {e}")
    
    finally:
        if listener is not None and listener.is_alive():
            listener.stop()
        running = False

if __name__ == "__main__":
    main()
