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
# 保证无论从哪个目录启动，都能以 SDK_PYTHON 包的相对方式导入 GentoRobot
if getattr(sys, "frozen", False):
    # 打包成 exe 后，配置都在 exe 所在目录
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(sys.executable))
else:
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_SCRIPT_DIR))
from SDK_PYTHON.GentoRobot import (
    GentoRobot,
    FXObjType,
    FXLogMask,
    state_map,
)

GRIPPER_CONFIG_PATH = os.path.join(_SCRIPT_DIR, "gripper_config.ini")
TELEOP_CONFIG_PATH = os.path.join(_SCRIPT_DIR, "teleop_config.ini")


def _load_teleop_config():
    config = configparser.ConfigParser()
    if os.path.isfile(TELEOP_CONFIG_PATH):
        try:
            config.read(TELEOP_CONFIG_PATH, encoding="utf-8")
        except Exception as exc:
            print(f"[配置] 读取失败，使用默认值: {exc}")
    return config


TELEOP_CFG = _load_teleop_config()


def _cfg_raw(section, key, fallback):
    try:
        return TELEOP_CFG.get(section, key)
    except (configparser.NoSectionError, configparser.NoOptionError):
        return fallback


def _cfg_float(section, key, fallback):
    try:
        return float(_cfg_raw(section, key, fallback))
    except (TypeError, ValueError):
        return fallback


def _cfg_int(section, key, fallback):
    try:
        return int(_cfg_raw(section, key, fallback))
    except (TypeError, ValueError):
        return fallback


def _cfg_bool(section, key, fallback):
    raw = _cfg_raw(section, key, None)
    if raw is None:
        return fallback
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


def _cfg_list(section, key, fallback, expected_len=None):
    raw = _cfg_raw(section, key, None)
    if raw is None:
        return list(fallback)
    try:
        values = [float(item) for item in raw.split(",") if item.strip()]
    except ValueError:
        return list(fallback)
    if not values or (expected_len is not None and len(values) != expected_len):
        return list(fallback)
    return values


ROBOT_IP = os.environ.get("GUJIAYAOCAO_ROBOT_IP") or _cfg_raw("robot", "ip", "192.168.1.190")


def _default_encoder_port():
    """未显式配置串口时，按平台自动选择。"""
    if os.name == "nt":
        ports = list(serial.tools.list_ports.comports())
        return ports[0].device if ports else "COM3"
    for candidate in ("/dev/ttyUSB0", "/dev/ttyACM0"):
        if os.path.exists(candidate):
            return candidate
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if ports else "/dev/ttyUSB0"


ENCODER_PORT = (
    os.environ.get("GUJIAYAOCAO_ENCODER_PORT")
    or str(_cfg_raw("encoder", "port", "") or "").strip()
    or _default_encoder_port()
)
ENCODER_BAUDRATE = int(
    os.environ.get("GUJIAYAOCAO_ENCODER_BAUDRATE")
    or _cfg_int("encoder", "baudrate", 1000000)
)

_DEFAULT_START_POSE_A = [120.0, -90.0, -90.0, -130.0, 0.0, 20.0, 0.0]
_DEFAULT_START_POSE_B = [-120.0, -90.0, 90.0, -130.0, 0.0, 20.0, 0.0]
START_POSE_A = _cfg_list("robot", "start_pose_a", _DEFAULT_START_POSE_A, 7)
START_POSE_B = _cfg_list("robot", "start_pose_b", _DEFAULT_START_POSE_B, 7)
START_POSE_TOL_DEG = _cfg_float("robot", "start_tol_deg", 5.0)
SKELETON_ALIGN_TOL_DEG = _cfg_float("robot", "skeleton_align_tol_deg", 10.0)
SKELETON_REALIGN_TOL_DEG = float(
    os.environ.get("GUJIAYAOCAO_SKELETON_REALIGN_TOL_DEG")
    or _cfg_float("robot", "skeleton_realign_tol_deg", 30.0)
)
MAIN_LOOP_PERIOD_SEC = max(0.001, _cfg_float("control", "loop_period_sec", 0.03))
GATE_STATUS_INTERVAL_SEC = max(0.0, _cfg_float("control", "gate_status_interval_sec", 0.05))
GATE_PRINT_INTERVAL_SEC = max(0.0, _cfg_float("control", "gate_print_interval_sec", 2.0))
HOME_WAIT_TIMEOUT_SEC = max(0.1, _cfg_float("control", "home_wait_timeout_sec", 10.0))
HOME_INTERP_EXTRA_SEC = max(0.0, _cfg_float("control", "home_interp_extra_sec", 5.0))
ROBOT_VEL_RATIO = _cfg_int("robot", "vel_ratio", 100)
ROBOT_ACC_RATIO = _cfg_int("robot", "acc_ratio", 100)
ENCODER_ENABLE_CHECKSUM = _cfg_bool("encoder", "enable_checksum", True)
TELEOP_START_STEP_DEG = float(
    os.environ.get("GUJIAYAOCAO_TELEOP_START_STEP_DEG")
    or _cfg_float("control", "teleop_start_step_deg", 0.2)
)
HOME_STEP_DEG = float(
    os.environ.get("GUJIAYAOCAO_HOME_STEP_DEG")
    or _cfg_float("control", "home_step_deg", 0.2)
)
CTRL_HZ = max(1.0, _cfg_float("control", "ctrl_hz", 100.0))
GRIPPER_CMD_HZ = float(
    os.environ.get("GUJIAYAOCAO_GRIPPER_CMD_HZ")
    or _cfg_float("gripper", "command_hz", 50.0)
)
GRIPPER_RECV_HZ = float(
    os.environ.get("GUJIAYAOCAO_GRIPPER_RECV_HZ")
    or _cfg_float("gripper", "recv_hz", 10.0)
)
ALIGN_STAGE_ARM = "arm"
ALIGN_STAGE_SKELETON = "skeleton"
ALIGN_STAGE_READY = "ready"

# 编码器 ID → 关节映射（默认按 1..16 顺序；实际接线不同时改 servo_config.ini 的 [encoder_map]）
DEFAULT_ARM_A_ENCODER_IDS = (1, 2, 3, 4, 5, 6, 7)       # A/L 臂 7 个关节
DEFAULT_LEFT_GRIPPER_ENCODER_ID = 8                      # 左夹爪
DEFAULT_ARM_B_ENCODER_IDS = (9, 10, 11, 12, 13, 14, 15)  # B/R 臂 7 个关节
DEFAULT_RIGHT_GRIPPER_ENCODER_ID = 16                    # 右夹爪
ENCODER_FRAME_COUNT = 16                                 # 一帧包含的编码器数量


# =====================【新增：导入达妙电机库】=====================
from KM_CAN import (
    Motor, KMGripperControl, KM_Motor_Type, ERROR_MESSAGES
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

# 脚踏按键映射：左=s，中=q，右=d（见 get_pedal_name）


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


def clear_pedal_latches():
    """清空踏板按下锁存，丢弃长操作期间的误触。"""
    with pedal_lock:
        for name in pedal_press_latch:
            pedal_press_latch[name] = False


def wrap_deg(deg):
    """把角度归一化到 [-180, 180)。"""
    return (deg + 180.0) % 360.0 - 180.0


def angle_diff_deg(a, b):
    """两角之间的最短路径差值（绝对值），避免 ±180° 折叠造成 360° 误差。"""
    return abs(wrap_deg(a - b))


def max_abs_joint_error(actual, target):
    return max(angle_diff_deg(actual[i], target[i]) for i in range(7))


def format_joint_list(joints):
    return "[" + ", ".join(f"{joint:.1f}" for joint in joints) + "]"


def step_joint_pose(current, target, max_step_deg):
    next_pose = []
    done = True
    for cur, dst in zip(current, target):
        delta = wrap_deg(dst - cur)
        if abs(delta) > max_step_deg:
            cur = wrap_deg(cur + math.copysign(max_step_deg, delta))
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
    start_err_a = [angle_diff_deg(a_real[i], START_POSE_A[i]) for i in range(7)]
    start_err_b = [angle_diff_deg(b_real[i], START_POSE_B[i]) for i in range(7)]
    check_skeleton = stage in (ALIGN_STAGE_SKELETON, ALIGN_STAGE_READY)
    skeleton_err_a = [angle_diff_deg(a_real[i], target_a[i]) for i in range(7)] if check_skeleton and filter_ok_a else [None] * 7
    skeleton_err_b = [angle_diff_deg(b_real[i], target_b[i]) for i in range(7)] if check_skeleton and filter_ok_b else [None] * 7
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

# =====================【Gento SDK：机器人初始化】=====================
ARM_A_OBJ = FXObjType.OBJ_ARM0
ARM_B_OBJ = FXObjType.OBJ_ARM1
ARM_OBJS = (ARM_A_OBJ, ARM_B_OBJ)
ARM_NAMES = {ARM_A_OBJ: "A", ARM_B_OBJ: "B"}

'''阻抗 / 拖动参数（teleop_config.ini 的 [impedance]）'''
# Gento PD 模式常用刚度/阻尼（配置缺失时用这组）
IMPEDANCE_K = _cfg_list("impedance", "k", [14, 14, 14, 10.5, 5.6, 5.6, 5.6], 7)
IMPEDANCE_D = _cfg_list("impedance", "d", [0.3] * 7, 7)
# 拖动模式（对齐/暂停）用较软的刚度，参考 Gento DragJoint 示例
DRAG_K = _cfg_list("impedance", "drag_k", [3, 3, 3, 2, 1, 1, 1], 7)
DRAG_D = _cfg_list("impedance", "drag_d", [0.2] * 7, 7)

'''PD 前馈周期（替代旧 SDK 的 set_vel_est_step）'''
ctrl_hz = CTRL_HZ
vel_step_ms = max(1, int(round(1000.0 / ctrl_hz)))

# 全局退出标志
running = True

'''初始化机器人接口'''
robot = GentoRobot()

'''查验连接是否成功（Gento SDK：link + check_link_state）'''
try:
    _ip_parts = [int(part) for part in str(ROBOT_IP).strip().split(".")]
    if len(_ip_parts) != 4:
        raise ValueError(f"IP 格式错误: {ROBOT_IP}")
except ValueError as exc:
    logger.error(f"failed:机器人 IP 配置错误：{exc}")
    sys.exit(0)

link_ret = robot.link(*_ip_parts, log_level=FXLogMask.FX_LOG_INFO_FLAG)
if not robot._connected:
    logger.error(f"failed:连接失败（{robot._get_operate_error_msg(link_ret)}）")
    sys.exit(0)

link_state = robot.check_link_state()
if link_state == 0:
    logger.error("failed:链路未建立")
    sys.exit(0)
if link_state == -1:
    logger.warning("warning:链路已建立，但 100ms 内未收到数据，请检查网线/防火墙")

motion_tag = 0
frame_update = None
for i in range(10):
    rt_dict = robot.get_rt_dict()
    frame_serial_value = rt_dict.get("frame_serial", 0) if isinstance(rt_dict, dict) else 0
    print(f"connect frames :{frame_serial_value}")
    if frame_serial_value != 0 and frame_update != frame_serial_value:
        motion_tag += 1
        frame_update = frame_serial_value
    time.sleep(0.1)
if motion_tag > 0:
    logger.info('success:机器人连接成功!')
else:
    logger.error('failed:机器人连接失败!')
    sys.exit(0)

'''开启日志以便检查'''
robot.set_log_level(FXLogMask.FX_LOG_INFO_FLAG)
try:
    logger.info(
        f"SDK版本: {robot.get_sdk_version()}，"
        f"控制器版本: {robot.get_controller_version()}"
    )
except Exception as exc:
    logger.warning(f"读取 SDK/控制器版本失败: {exc}")


def _arm_state_name(obj_type):
    return state_map.get(robot.current_state(obj_type), "Unknown")


def _arm_low_speed_flag(obj_type):
    """到位/低速标志（SG 慢组数据，1kHz 实时组不含）。"""
    idx = 0 if obj_type == ARM_A_OBJ else 1
    return int(robot.sg.m_ARMS[idx].m_ARM_GET.m_ARM_FBK_LowSpdFlag)


def reset_arm_if_error():
    """仅错误状态复位；正常状态不动。"""
    for obj_type in ARM_OBJS:
        if _arm_state_name(obj_type) == "Error":
            ret, system_error = robot.reset_error(obj_type, 5000)
            logger.info(f"[ROBOT] {ARM_NAMES[obj_type]} 臂复位错误 ret={ret} err={system_error}")
            time.sleep(0.2)


def _reset_one_arm(obj_type):
    """单个手臂清错 / 回到 IDLE。"""
    state_name = _arm_state_name(obj_type)
    if state_name == "Error":
        ret, system_error = robot.reset_error(obj_type, 5000)
        logger.info(f"[ROBOT] {ARM_NAMES[obj_type]} 臂复位错误 ret={ret} err={system_error}")
        time.sleep(0.2)
    elif state_name not in ("IDLE", "Unknown"):
        ret = robot.switch_to_idle(obj_type, 3000)
        logger.info(f"[ROBOT] {ARM_NAMES[obj_type]} 臂切到 IDLE ret={ret}")
        time.sleep(0.1)


def reset_arm_errors():
    """双臂清错 / 回到 IDLE：Gento 状态切换要求先 IDLE。"""
    for obj_type in ARM_OBJS:
        _reset_one_arm(obj_type)


def _switch_mode(obj_type, switch_func, *args, label=""):
    """切换控制模式；失败时先复位/回 IDLE 再试一次（只动当前臂）。"""
    ret = switch_func(obj_type, *args)
    if ret == 0:
        return 0
    logger.warning(
        f"[ROBOT] {ARM_NAMES[obj_type]} 切到 {label} 失败 ret={ret}"
        f"（{robot._get_operate_error_msg(ret)}），复位后重试"
    )
    _reset_one_arm(obj_type)
    return switch_func(obj_type, *args)


def set_joint_drag_mode(enable: bool):
    """enable=True 进关节拖动（对齐/暂停）；False 进 PD 遥操模式。"""
    target = "DragJoint" if enable else "PD"
    for obj_type in ARM_OBJS:
        if _arm_state_name(obj_type) == target:
            continue
        if enable:
            ret = _switch_mode(
                obj_type, robot.switch_to_drag_joint, 3000,
                DRAG_K, DRAG_D, label="DragJoint",
            )
        else:
            ret = _switch_mode(
                obj_type, robot.switch_to_pd_mode, 3000,
                ROBOT_VEL_RATIO, ROBOT_ACC_RATIO,
                IMPEDANCE_K, IMPEDANCE_D, label="PD",
            )
        if ret != 0:
            logger.error(
                f"[ROBOT] {ARM_NAMES[obj_type]} 模式切换失败 ret={ret}"
                f"（{robot._get_operate_error_msg(ret)}）"
            )
    print(f"[DRAG] joint drag {'enabled' if enable else 'disabled'}")


reset_arm_errors()
time.sleep(0.5)


# =====================【正确设置工具负载】======================
# 参数在 teleop_config.ini 的 [tool] 中配置
# kineParams [X,Y,Z mm, A,B,C deg] 法兰到工具末端偏移
_DEFAULT_TOOL_KINE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
_DEFAULT_TOOL_DYN = [0.5, 0.0, 0.0, 50.0, 0.004, 0.000, 0.00, 0.004, 0.000, 0.03]
tool_kine_A = _cfg_list("tool", "a_kine", _DEFAULT_TOOL_KINE, 6)
tool_dyn_A = _cfg_list("tool", "a_dyn", _DEFAULT_TOOL_DYN, 10)
tool_kine_B = _cfg_list("tool", "b_kine", _DEFAULT_TOOL_KINE, 6)
tool_dyn_B = _cfg_list("tool", "b_dyn", _DEFAULT_TOOL_DYN, 10)
ret_a = robot.runtime_set_tool_kd(ARM_A_OBJ, tool_kine_A, tool_dyn_A)
ret_b = robot.runtime_set_tool_kd(ARM_B_OBJ, tool_kine_B, tool_dyn_B)
logger.info(f"设置工具 A={ret_a}, B={ret_b} | 0=成功")
if ret_a != 0 or ret_b != 0:
    logger.warning("警告：工具设置返回非0，阻抗动力学补偿可能失效！")
time.sleep(0.3)

'''PD 前馈周期（遥操/回Home 都以 PD 模式流式下发）'''
pd_ret = robot.config_set_pd_cmd_cycle_time(vel_step_ms)
logger.info(f"PD 前馈周期{vel_step_ms}ms（ctrl_hz={ctrl_hz:g}），设置结果={pd_ret}")
time.sleep(0.2)

'''进入拖动模式，等待人工对齐'''
set_joint_drag_mode(True)
logger.info(
    f"已进入关节拖动模式。请手动拖动到：L/A={format_joint_list(START_POSE_A)}，"
    f"R/B={format_joint_list(START_POSE_B)}，误差需在±{START_POSE_TOL_DEG:.1f}°内。"
)
time.sleep(0.5)


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
                    if len(response) >= ENCODER_FRAME_COUNT * 8:
                        break
                else:
                    no_data_count += 1
                    time.sleep(0.001)
                    # 已收到部分数据但缺尾，等 5ms 仍不齐则放弃（避免拖慢 100Hz 循环）
                    if response and no_data_count >= 5:
                        break
        return response if response else None


class EncoderReader:
    """编码器核心处理类，增加一阶低通滤波 + 夹爪遥操作"""
    def __init__(self):
        # ===================== 可配置参数（teleop_config.ini） =====================
        self.port = ENCODER_PORT
        self.baudrate = ENCODER_BAUDRATE
        self.send_frequency = _cfg_float("encoder", "send_frequency", 100.0)
        self.enable_checksum = ENCODER_ENABLE_CHECKSUM
        self.encoder_resolution = int(_cfg_float("encoder", "resolution", 4096))

        # 拼接得到ini完整绝对路径，固定地址
        self.config_file = os.path.join(_SCRIPT_DIR, "servo_config.ini")

        # 校验配置文件是否存在，不存在抛出异常终止程序，打印完整路径方便排查
        if not os.path.isfile(self.config_file):
            raise FileNotFoundError(
                f"伺服配置文件缺失！\n预期路径：{self.config_file}\n请确认servo_config.ini放置到脚本同级目录"
            )
        # 一阶低通滤波系数 α 0~1；越小滤波越强，响应越慢
        self.filter_alpha = _cfg_float("encoder", "filter_alpha", 0.9)

        # ----------------【夹爪配置（teleop_config.ini 的 [gripper]）】----------------
        # 编码器 ID 与关节的对应关系（可在 servo_config.ini 的 [encoder_map] 覆盖）
        self.arm_a_ids = list(DEFAULT_ARM_A_ENCODER_IDS)
        self.arm_b_ids = list(DEFAULT_ARM_B_ENCODER_IDS)
        self.left_gripper_id = DEFAULT_LEFT_GRIPPER_ENCODER_ID
        self.right_gripper_id = DEFAULT_RIGHT_GRIPPER_ENCODER_ID
        # STS编码器摆动角度范围(deg)
        self.enc_min_deg = _cfg_float("gripper", "enc_min_deg", 0.0)
        self.enc_max_deg = _cfg_float("gripper", "enc_max_deg", 90.0)
        # 达妙DM4310夹爪电机CAN ID
        self.motor_left_slave  = _cfg_int("gripper", "left_motor_id", 1)
        self.motor_right_slave = _cfg_int("gripper", "right_motor_id", 2)
        self.motor_left_master  = self.motor_left_slave + 0x10
        self.motor_right_master = self.motor_right_slave + 0x10
        # MIT控制参数
        self.grip_kp = _cfg_float("gripper", "kp", 3.0)
        self.grip_kd = _cfg_float("gripper", "kd", 0.12)
        self.grip_tau_ff = _cfg_float("gripper", "tau_ff", 0.0)
        # 夹爪电机机械限位 rad
        self.gripper_qmin = _cfg_float("gripper", "qmin", 0.0)
        self.gripper_qmax = _cfg_float("gripper", "qmax", 1.3)
        self.gripper_teleop_enable = _cfg_bool("gripper", "teleop_enable", True)   # 是否开启夹爪跟随
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
        # RLock：parse_data 持锁写角度时，内部还会调用 get_accumulated_position 再次取锁
        self.data_mutex = threading.RLock()

        # 按编码器 ID 存储滤波结果，避免依赖帧内顺序
        self.filtered_angle: Dict[int, float] = {}
        self.filter_init_flag: Dict[int, bool] = {}

        #初始化串口驱动
        self.driver = STSDriver(self.port, self.baudrate)
        self.load_config()
        self.load_gripper_config()

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

    def load_gripper_config(self):
        """读取夹爪端点映射（gripper_config.ini），按实际闭合/张开原始值归一化。"""
        self.gripper_config = {}
        if not os.path.isfile(GRIPPER_CONFIG_PATH):
            return
        config = configparser.ConfigParser()
        try:
            config.read(GRIPPER_CONFIG_PATH, encoding="utf-8")
        except Exception as exc:
            print(f"[夹爪配置] 读取失败，使用旧的 abs/9 归一化: {exc}")
            return
        for side, sid in (("left", self.left_gripper_id), ("right", self.right_gripper_id)):
            if not config.has_section(side):
                continue
            try:
                closed_raw = int(config[side]["closed_raw"])
                open_raw = int(config[side]["open_raw"])
            except (KeyError, ValueError):
                continue
            if closed_raw == open_raw:
                continue
            self.gripper_config[side] = {
                "sid": sid,
                "closed_raw": closed_raw,
                "open_raw": open_raw,
                "target_closed": float(config[side].get("target_closed", "-0.15")),
                "target_open": float(config[side].get("target_open", "1.1")),
            }
        if self.gripper_config:
            parts = [
                f"{side}(ID{cfg['sid']}) 张开={cfg['open_raw']} 闭合={cfg['closed_raw']}"
                for side, cfg in self.gripper_config.items()
            ]
            print("[夹爪配置] 已加载: " + "; ".join(parts))
        else:
            print("[夹爪配置] 未找到有效端点，夹爪使用旧的 abs(角度)/9 归一化")

    def _raw_to_relative_deg(self, sid: int, raw: int) -> float:
        """原始值换算成与 filtered_angle 一致的相对角度（deg）。"""
        zero = self.zero_offset_raw.get(sid, 0)
        direction = self.direction.get(sid, 1)
        return wrap_deg((raw - zero) * direction * 360.0 / self.encoder_resolution)

    def _gripper_norm(self, side: str, deg: float) -> float:
        """夹爪张开程度 0(闭合)~1(张开)；无配置时退回 abs(deg)/9。"""
        cfg = self.gripper_config.get(side)
        if not cfg:
            return max(0.0, min(1.0, abs(deg) / 9.0))
        closed_deg = self._raw_to_relative_deg(cfg["sid"], cfg["closed_raw"])
        open_deg = self._raw_to_relative_deg(cfg["sid"], cfg["open_raw"])
        span = open_deg - closed_deg
        if abs(span) < 1e-6:
            return max(0.0, min(1.0, abs(deg) / 9.0))
        return max(0.0, min(1.0, (deg - closed_deg) / span))

    def _gripper_target(self, side: str, norm: float) -> float:
        """张开程度换算成 MIT 位置目标（rad）。"""
        cfg = self.gripper_config.get(side)
        if not cfg:
            return -0.15 + norm * (1.1 - (-0.15))
        closed_target = cfg["target_closed"]
        open_target = cfg["target_open"]
        return closed_target + norm * (open_target - closed_target)

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
        # Gento SDK：RT 数据由底层 1kHz 刷新，直接读结构体即可
        rt = robot.rt
        arm_a_out = rt.m_ARMS[0].m_ARM_OUT
        arm_b_out = rt.m_ARMS[1].m_ARM_OUT
        a_real = [arm_a_out.m_ARM_FBK_Joint_Pos[i] for i in range(7)]
        b_real = [arm_b_out.m_ARM_FBK_Joint_Pos[i] for i in range(7)]
        return a_real, b_real

    def get_send_frequency(self):
        return self.send_frequency

    @staticmethod
    def _parse_id_list(text, fallback, expected_len):
        """解析 "1,2,3..." 形式的 ID 列表，格式不对则回退默认值。"""
        try:
            ids = [int(item.strip()) for item in text.split(",") if item.strip()]
        except ValueError:
            return list(fallback)
        if len(ids) != expected_len:
            return list(fallback)
        return ids

    @staticmethod
    def _parse_id(text, fallback):
        try:
            return int(str(text).strip())
        except (TypeError, ValueError):
            return fallback

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
            except Exception:
                continue
        if config.has_section("encoder_map"):
            mapping = config["encoder_map"]
            self.arm_a_ids = self._parse_id_list(
                mapping.get("arm_a", ""), DEFAULT_ARM_A_ENCODER_IDS, 7)
            self.arm_b_ids = self._parse_id_list(
                mapping.get("arm_b", ""), DEFAULT_ARM_B_ENCODER_IDS, 7)
            self.left_gripper_id = self._parse_id(
                mapping.get("left_gripper", ""), DEFAULT_LEFT_GRIPPER_ENCODER_ID)
            self.right_gripper_id = self._parse_id(
                mapping.get("right_gripper", ""), DEFAULT_RIGHT_GRIPPER_ENCODER_ID)
        print(f"[配置] 已加载: {self.config_file}")
        print(
            f"[配置] 编码器映射 A臂={self.arm_a_ids} 左夹爪={self.left_gripper_id} "
            f"B臂={self.arm_b_ids} 右夹爪={self.right_gripper_id}"
        )

    def save_config(self):
        config = configparser.ConfigParser()
        all_ids = set(self.zero_offset_raw.keys()) | set(self.direction.keys())
        for sid in all_ids:
            config[str(sid)] = {
                "zero_offset_raw": str(self.zero_offset_raw.get(sid, 0)),
                "direction": str(self.direction.get(sid, 1))
            }
        config["encoder_map"] = {
            "arm_a": ",".join(str(i) for i in self.arm_a_ids),
            "left_gripper": str(self.left_gripper_id),
            "arm_b": ",".join(str(i) for i in self.arm_b_ids),
            "right_gripper": str(self.right_gripper_id)
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
                if sid==9:
                    #self.alignment_summary_var.set(f"raw = {raw}, accumulated_position={self.accumulated_position[sid]}, zero_offset_raw={self.zero_offset_raw.get(sid, 0)}")
                    print(f"raw = {raw}, accumulated_position={self.accumulated_position[sid]}, zero_offset_raw={self.zero_offset_raw.get(sid, 0)}")
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

    def _scan_frame_offsets(self, data: List[int]) -> List[int]:
        """在字节流中按 FF FF 帧头 + 校验和寻找 8 字节帧，自动跳过噪声和残缺帧。"""
        offsets = []
        i = 0
        limit = len(data) - 8
        while i <= limit:
            if data[i] != 0xFF or data[i+1] != 0xFF:
                i += 1
                continue
            if self.enable_checksum and not self.verify_checksum(data, i):
                i += 1
                continue
            offsets.append(i)
            i += 8
        return offsets

    def parse_data(self, data: List[int]):
        """解析编码器 + 一阶低通滤波（写入受 data_mutex 保护）"""
        offsets = self._scan_frame_offsets(data)
        if len(offsets) < ENCODER_FRAME_COUNT:
            return

        with self.data_mutex:
            for off in offsets[:ENCODER_FRAME_COUNT]:
                sid = data[off+2]
                raw = (data[off+6] <<8) | data[off+5]
                rad = self.get_radian(sid, raw)
                deg = rad * 180 / math.pi

                # ========= 一阶低通滤波 y = α*x + (1-α)*y_prev =========
                if not self.filter_init_flag.get(sid):
                    self.filtered_angle[sid] = deg
                    self.filter_init_flag[sid] = True
                else:
                    self.filtered_angle[sid] = self.filter_alpha * deg + (1.0 - self.filter_alpha) * self.filtered_angle.get(sid, deg)

    def read_loop(self):
        global teleop_interp_a, teleop_interp_b
        while running:
            period = 1.0 / self.get_send_frequency()
            start = time.time()
            res = self.driver.send_and_receive(self.cmd)
            if res:
                # ✅无论遥操作开不开，永远解析编码器，刷新filtered_angle
                self.parse_data(res)

                # =========只有遥操作开启 going=False，才下发从手机械臂指令=========
                if not going:
                    # 使用【滤波后】角度下发机械臂本体（按编码器 ID 映射，不依赖帧内顺序）
                    with self.data_mutex:
                        joint_cmd_1 = [self.filtered_angle.get(sid, 0.0) for sid in self.arm_a_ids]
                        joint_cmd_2 = [self.filtered_angle.get(sid, 0.0) for sid in self.arm_b_ids]
                        grip_l_deg = self.filtered_angle.get(self.left_gripper_id, 0.0)
                        grip_r_deg = self.filtered_angle.get(self.right_gripper_id, 0.0)
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
                    # Gento PD 模式：直接下发关节位置指令（无需 clear_set/send_cmd 批量组包）
                    robot.runtime_set_joint_pos_pd_cmd(ARM_A_OBJ, joint_cmd_1)
                    robot.runtime_set_joint_pos_pd_cmd(ARM_B_OBJ, joint_cmd_2)
                    # =====================【夹爪遥操作逻辑】=====================
                    now = time.monotonic()
                    if self.gripper_teleop_enable and now >= self.next_gripper_cmd_time:
                        self.next_gripper_cmd_time = now + self.gripper_cmd_period
                        q_l_norm = self._gripper_norm("left", grip_l_deg)
                        q_r_norm = self._gripper_norm("right", grip_r_deg)
                        q_l_target = self._gripper_target("left", q_l_norm)
                        q_r_target = self._gripper_target("right", q_r_norm)
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

                # Gento SDK 的 RT 数据由底层线程刷新，编码器线程只负责下发 PD 指令
            # 周期控频，保证循环持续跑
            elapsed = time.time() - start
            sleep_time = max(0, period - elapsed)
            time.sleep(sleep_time)

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
        t1.start()

        # 后台启动脚踏键盘监听
        listener = keyboard.Listener(on_press=on_key_press, on_release=on_key_release)
        listener.daemon = True
        listener.start()
        print("✅脚踏监听启动：s左踏板按下=回Home；d右踏板按下瞬间=开启摇操，ESC退出")
        print(f"[GATE] 第一步：先对齐机械臂到 L/A={format_joint_list(START_POSE_A)}，R/B={format_joint_list(START_POSE_B)}，允许误差±{START_POSE_TOL_DEG:.1f}°。")
        print(f"[GATE] 第二步：机械臂对齐后，再对齐骨架编码器；每个骨架关节只需进入一次≤{SKELETON_ALIGN_TOL_DEG:.1f}°误差范围。")
        last_gate_status_time = 0.0
        last_gate_print_time = 0.0
        skeleton_aligned_a = [False] * 7
        skeleton_aligned_b = [False] * 7
        teleop_has_started = False
        skeleton_realign_required = False
        # =========主程序循环========================

        # 主循环状态（遥操中不再刷新，保留最后一次对齐结果）
        a_real = [0.0] * 7
        b_real = [0.0] * 7
        target_A = [0.0] * 7
        target_B = [0.0] * 7
        filter_ok_a = False
        filter_ok_b = False
        max_err_a = 999.0
        max_err_b = 999.0
        max_err = 999.0
        start_err_a = 999.0
        start_err_b = 999.0
        start_pose_ok = False
        can_start_teleop = False
        alignment_stage = ALIGN_STAGE_ARM

        while running:
            loop_start = time.time()
            # 消费脚踏按下事件。事件由键盘回调锁存，短脉冲不会被主循环漏掉。
            left_pressed = consume_pedal_press("left")
            right_pressed = consume_pedal_press("right")

            if going:
                # 对齐阶段：实时检查；Gento 的 RT 由底层 1kHz 刷新，直接读即可
                a_real, b_real = reader.get_robot_real_joint()

                # =====================【持续实时对比：机械臂真实角度 VS 编码器角度】=====================
                MAX_ALLOW_ERR_DEG = SKELETON_ALIGN_TOL_DEG   # 最大允许单关节角度误差(度)

                filter_ok_a = False
                filter_ok_b = False
                max_err_a = 999.0
                max_err_b = 999.0
                with reader.data_mutex:
                    target_A = [reader.filtered_angle.get(sid, 0.0) for sid in reader.arm_a_ids]
                    target_B = [reader.filtered_angle.get(sid, 0.0) for sid in reader.arm_b_ids]
                    filter_ok_a = all(reader.filter_init_flag.get(sid, False) for sid in reader.arm_a_ids)
                    filter_ok_b = all(reader.filter_init_flag.get(sid, False) for sid in reader.arm_b_ids)

                start_err_a = max_abs_joint_error(a_real, START_POSE_A)
                start_err_b = max_abs_joint_error(b_real, START_POSE_B)
                start_pose_ok = start_err_a <= START_POSE_TOL_DEG and start_err_b <= START_POSE_TOL_DEG
                if not start_pose_ok and not teleop_has_started and not skeleton_realign_required:
                    skeleton_aligned_a = [False] * 7
                    skeleton_aligned_b = [False] * 7

                # 只有初始化完成才计算误差，否则误差置很大
                if filter_ok_a and filter_ok_b:
                    skeleton_err_a = [angle_diff_deg(a_real[i], target_A[i]) for i in range(7)]
                    skeleton_err_b = [angle_diff_deg(b_real[i], target_B[i]) for i in range(7)]
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
                    filter_ok_a
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
                if now - last_gate_status_time >= GATE_STATUS_INTERVAL_SEC:
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
                    last_gate_status_time = now

                # 控制台打印单独节流，避免 20Hz 状态刷新刷屏
                if now - last_gate_print_time >= GATE_PRINT_INTERVAL_SEC:
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
                    last_gate_print_time = now

                # 【可选】打开实时偏差打印，调试用；正式运行可以注释掉
                #print(f"实时偏差 | A臂={max_err_a:.2f}°, B臂={max_err_b:.2f}°,最大={max_err:.2f}°")
                # ====================================================================================
            # =========左踏板s：按下执行回Home=========
            if left_pressed is True and home_running is False:
                if not going:
                    # 遥操中按下：直接读当前姿态
                    a_real, b_real = reader.get_robot_real_joint()
                going = True
                teleop_interp_a = None
                teleop_interp_b = None
                home_running = True
                # 丢弃回Home过程中被锁存的踏板误触，避免结束后立刻触发遥操
                clear_pedal_latches()
                print("【左踏板按下】执行回Home点位")
                reset_arm_if_error()
                # 回Home期间切到 PD 模式，禁止手动拖动，到位后也保持不可拖动
                set_joint_drag_mode(False)
                time.sleep(0.2)

                joint_cmd_1 = list(a_real)
                joint_cmd_2 = list(b_real)
                home_period = 1.0 / reader.get_send_frequency()
                max_home_delta = max(
                    max_abs_joint_error(joint_cmd_1, START_POSE_A),
                    max_abs_joint_error(joint_cmd_2, START_POSE_B),
                )
                home_interp_timeout = max(10.0, max_home_delta / max(HOME_STEP_DEG, 0.001) * home_period + HOME_INTERP_EXTRA_SEC)
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

                    robot.runtime_set_joint_pos_pd_cmd(ARM_A_OBJ, joint_cmd_1)
                    robot.runtime_set_joint_pos_pd_cmd(ARM_B_OBJ, joint_cmd_2)

                    if done_a and done_b:
                        break
                    if (time.time() - home_interp_start) > home_interp_timeout:
                        print(f"⚠️回Home插点超时({home_interp_timeout:.1f}s)，停止继续插点")
                        break
                    time.sleep(home_period)

                # 等待双臂运动到位，增加超时保护
                MAX_WAIT_SEC = HOME_WAIT_TIMEOUT_SEC
                start_wait = time.time()

                while True:
                    flag_a = _arm_low_speed_flag(ARM_A_OBJ)
                    flag_b = _arm_low_speed_flag(ARM_B_OBJ)
                    print(f"A臂low_speed_flag={flag_a}, B臂low_speed_flag={flag_b}")

                    if flag_a == 1 and flag_b == 1:
                        print("回home点完成！！")
                        break
                    if (time.time() - start_wait) > MAX_WAIT_SEC:
                        print(f"⚠️回Home等待超时({MAX_WAIT_SEC}s)！机械臂未收到到位信号，请检查机械臂状态！")
                        break
                    time.sleep(1.5)
                home_running = False
                teleop_has_started = False
                skeleton_realign_required = False
                skeleton_aligned_a = [False] * 7
                skeleton_aligned_b = [False] * 7
                clear_pedal_latches()
                continue
                # 右踏板 d：边沿触发，按下瞬间开启摇操遥操作
            if right_pressed is True:
                if not going:
                    going = True
                    teleop_interp_a = None
                    teleop_interp_b = None
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

            # 控制主循环周期
            elapsed = time.time() - loop_start
            if elapsed < MAIN_LOOP_PERIOD_SEC:
                time.sleep(MAIN_LOOP_PERIOD_SEC - elapsed)

        # ----------------ESC按下后执行资源清理（只保留一份）----------------
        if reader:
            time.sleep(0.3)
            for obj_type in ARM_OBJS:
                robot.switch_to_idle(obj_type, 3000)
            time.sleep(0.2)
            print("程序正常退出！！")
    except Exception as e:
        print(f"程序启动失败: {e}")

    finally:
        if listener is not None and listener.is_alive():
            listener.stop()
        running = False
        going = True   # 先停止编码器线程继续下发
        time.sleep(0.1)
        # Gento SDK：退出前切 IDLE、断开连接并释放句柄
        try:
            for obj_type in ARM_OBJS:
                robot.switch_to_idle(obj_type, 2000)
        except Exception:
            pass
        try:
            robot.unlink()
        except Exception:
            pass
        try:
            robot.cleanup()
        except Exception:
            pass

if __name__ == "__main__":
    main()
