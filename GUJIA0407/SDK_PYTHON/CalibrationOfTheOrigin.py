# -*- coding: utf-8 -*-
"""
骨架舵机原点标定程序

功能：
- 通过串口读取 16 路 STS 舵机编码器的原始值（0~4095）
- 操作人员把每个关节分别转到机械正限位、负限位，程序逐个记录两端原始值
- 连续跟踪编码器运动增量，按实际运动路径计算中点，超过 180° 也能正确计算
- 按关节额定范围折算原点：对称关节取中点；非对称关节（四轴 A4/B4，
  额定 -145°~+60°）按额定范围把零点偏移到正确位置
- 由操作人员在前端逐个确认，标定结果导出为独立的 .ini 文件

原点计算：
    对称关节   zero = 路径中点
    非对称关节 zero = 路径中点 - direction * (min_deg + max_deg) / 2 / 每格角度
    额定范围来自内置默认值，可被 teleop_config.ini 的 [joint_range] 覆盖。

重要：servo_config.ini 只读取 [encoder_map] 和各 ID 的 direction，本程序
      不会写入或修改 servo_config.ini。

与摇操程序（gujiayaocao_0817.py）完全独立，可单独运行。
"""
import configparser
import math
import os
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    import serial.tools.list_ports
    SERIAL_IMPORT_ERROR = None
except Exception as exc:  # pyserial 缺失时界面仍可打开，连接时提示
    serial = None
    SERIAL_IMPORT_ERROR = exc


if getattr(sys, "frozen", False):
    SCRIPT_DIR = os.path.dirname(os.path.abspath(sys.executable))
else:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SERVO_CONFIG_PATH = os.path.join(SCRIPT_DIR, "servo_config.ini")
TELEOP_CONFIG_PATH = os.path.join(SCRIPT_DIR, "teleop_config.ini")

ENCODER_RESOLUTION = 4096
ENCODER_FRAME_COUNT = 16
READ_HZ = 50.0
SMOOTH_WINDOW = 12          # 记录限位时用于平滑的采样点数
MIN_SMOOTH_SAMPLES = 5      # 至少收到多少个点才允许记录
MIN_TRAVEL_COUNTS = 8       # 行程过小视为异常（约 0.7°）
STABLE_TOL_COUNTS = 8       # 记录前最近采样允许的抖动（约 0.7°）
TRAVEL_LIMIT_COUNTS = ENCODER_RESOLUTION  # 行程上限（一圈）

READ_COMMAND = [
    0xFF, 0xFF, 0xFE, 0x14, 0x82, 0x38, 0x02,
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0xA9,
]

# 编码器 ID 默认映射（与 servo_config.ini 的 [encoder_map] 一致）
DEFAULT_ARM_A_IDS = (1, 2, 3, 4, 5, 6, 7)
DEFAULT_LEFT_GRIPPER_ID = 8
DEFAULT_ARM_B_IDS = (9, 10, 11, 12, 13, 14, 15)
DEFAULT_RIGHT_GRIPPER_ID = 16

# 关节空间额定范围（度，min, max）。对称关节不需要配置（原点=中点）；
# 四轴 A4/B4 机械范围 -145°~+60°，中点不是零点，必须配置。
# teleop_config.ini 的 [joint_range] 可覆盖这里的默认值。
DEFAULT_JOINT_RANGES = {
    "A4": (-145.0, 60.0),
    "B4": (-145.0, 60.0),
}
RANGE_TOL_DEG = 10.0        # 实测行程与额定行程允许的偏差（度）


@dataclass
class JointSpec:
    label: str
    sid: int
    direction: Optional[int] = None     # 来自 servo_config.ini 的 direction（1/-1）
    min_deg: Optional[float] = None     # 额定负限位（关节空间）
    max_deg: Optional[float] = None     # 额定正限位（关节空间）

    @property
    def rated_travel_deg(self) -> Optional[float]:
        if self.min_deg is None or self.max_deg is None:
            return None
        return self.max_deg - self.min_deg


@dataclass
class JointCalibration:
    spec: JointSpec
    pos_limit: Optional[float] = None
    neg_limit: Optional[float] = None
    pos_path: Optional[float] = None   # 记录正限位时的累计路径（展开值）
    neg_path: Optional[float] = None   # 记录负限位时的累计路径（展开值）
    origin: Optional[float] = None
    travel: Optional[int] = None
    travel_deg: Optional[float] = None        # 负限位→正限位的有符号关节行程，应为正
    rated_travel_deg: Optional[float] = None  # 额定行程（未配置范围时为 None）
    direction_used: int = 1                   # 计算/显示实际使用的方向
    direction_inferred: Optional[int] = None  # 按记录推断出的方向
    direction_mismatch: bool = False          # 配置方向与记录矛盾（正负限位记反）
    confirmed: bool = False

    @property
    def sid(self) -> int:
        return self.spec.sid

    @property
    def label(self) -> str:
        return self.spec.label

    @property
    def rated_travel_error_deg(self) -> Optional[float]:
        if self.travel_deg is None or self.rated_travel_deg is None:
            return None
        return self.travel_deg - self.rated_travel_deg


# ===================== 纯函数 =====================

def _parse_id_list(text, fallback, expected_len):
    try:
        ids = [int(item.strip()) for item in text.split(",") if item.strip()]
    except ValueError:
        return list(fallback)
    if len(ids) != expected_len:
        return list(fallback)
    return ids


def _parse_id(text, fallback):
    try:
        return int(str(text).strip())
    except (TypeError, ValueError):
        return fallback


def _parse_range(text) -> Optional[tuple]:
    """解析 'min,max'（度，关节空间）；格式无效返回 None。"""
    parts = str(text).replace("，", ",").split(",")
    try:
        lo, hi = (float(part.strip()) for part in parts)
    except (TypeError, ValueError):
        return None
    if not (-180.0 <= lo < hi <= 180.0):
        return None
    return lo, hi


def load_joint_ranges() -> Dict[str, tuple]:
    """读取 teleop_config.ini 的 [joint_range]：键为关节名或编码器 ID，值为 'min,max'。"""
    ranges: Dict[str, tuple] = {}
    if not os.path.isfile(TELEOP_CONFIG_PATH):
        return ranges
    config = configparser.ConfigParser()
    config.optionxform = str      # 保留 A4 / B4 的大小写
    try:
        config.read(TELEOP_CONFIG_PATH, encoding="utf-8")
    except Exception as exc:
        print(f"[配置] joint_range 读取失败，使用内置范围: {exc}")
        return ranges
    if not config.has_section("joint_range"):
        return ranges
    for key, value in config["joint_range"].items():
        parsed = _parse_range(value)
        if parsed is None:
            print(f"[配置] joint_range 中 {key}={value!r} 无效，已忽略")
            continue
        ranges[str(key).strip().upper()] = parsed
    return ranges


def load_joint_specs() -> List[JointSpec]:
    """读取 servo_config.ini 的 [encoder_map]/direction 和关节额定范围。

    servo_config.ini 缺失或格式不对时用默认映射；额定范围 = 内置默认值，
    再被 teleop_config.ini 的 [joint_range] 覆盖。
    """
    arm_a = list(DEFAULT_ARM_A_IDS)
    arm_b = list(DEFAULT_ARM_B_IDS)
    left_gripper = DEFAULT_LEFT_GRIPPER_ID
    right_gripper = DEFAULT_RIGHT_GRIPPER_ID
    directions: Dict[int, int] = {}

    if os.path.isfile(SERVO_CONFIG_PATH):
        config = configparser.ConfigParser()
        try:
            config.read(SERVO_CONFIG_PATH, encoding="utf-8")
        except Exception as exc:
            print(f"[配置] 读取失败，使用默认映射: {exc}")
        else:
            if config.has_section("encoder_map"):
                mapping = config["encoder_map"]
                arm_a = _parse_id_list(mapping.get("arm_a", ""), DEFAULT_ARM_A_IDS, 7)
                arm_b = _parse_id_list(mapping.get("arm_b", ""), DEFAULT_ARM_B_IDS, 7)
                left_gripper = _parse_id(mapping.get("left_gripper", ""), DEFAULT_LEFT_GRIPPER_ID)
                right_gripper = _parse_id(mapping.get("right_gripper", ""), DEFAULT_RIGHT_GRIPPER_ID)
            for section in config.sections():
                try:
                    sid = int(section)
                except ValueError:
                    continue
                try:
                    value = int(str(config[section]["direction"]).strip())
                except (KeyError, ValueError):
                    continue
                if value in (1, -1):
                    directions[sid] = value

    ranges = dict(DEFAULT_JOINT_RANGES)
    ranges.update(load_joint_ranges())

    joints = [(f"A{i + 1}", sid) for i, sid in enumerate(arm_a)]
    joints.append(("左夹爪", left_gripper))
    joints.extend((f"B{i + 1}", sid) for i, sid in enumerate(arm_b))
    joints.append(("右夹爪", right_gripper))

    specs = []
    for label, sid in joints:
        spec = JointSpec(label, sid, directions.get(sid))
        rng = ranges.get(label.upper())
        if rng is None:
            rng = ranges.get(str(sid).upper())
        if rng is not None:
            spec.min_deg, spec.max_deg = rng
        specs.append(spec)
    return specs


def circular_mean(samples) -> Optional[float]:
    """编码器原始值的圆周平均，正确处理 0/4095 附近的角度回绕。"""
    if not samples:
        return None
    scale = 2.0 * math.pi / ENCODER_RESOLUTION
    sin_sum = sum(math.sin(value * scale) for value in samples)
    cos_sum = sum(math.cos(value * scale) for value in samples)
    if abs(sin_sum) < 1e-12 and abs(cos_sum) < 1e-12:
        return float(samples[-1])
    return (math.atan2(sin_sum, cos_sum) / scale) % ENCODER_RESOLUTION


def counts_to_deg(counts: float) -> float:
    return counts * 360.0 / ENCODER_RESOLUTION


def joint_deg(raw: float, origin: float, direction: int) -> float:
    """原始值相对标定原点的关节角度（度，已按 direction 换算），归一化到 [-180, 180)。"""
    offset = (raw - origin + ENCODER_RESOLUTION / 2.0) % ENCODER_RESOLUTION - ENCODER_RESOLUTION / 2.0
    return counts_to_deg(offset) * direction


def unwrap_delta(new_raw: int, last_raw: int) -> int:
    """相邻两帧原始值之差，折算到最短方向（[-2048, 2048)），用于连续累计路径。"""
    delta = (new_raw - last_raw) % ENCODER_RESOLUTION
    if delta > ENCODER_RESOLUTION // 2:
        delta -= ENCODER_RESOLUTION
    return delta


def is_same_file(path_a: str, path_b: str) -> bool:
    """判断两个路径是否指向同一文件（兼容 Windows 大小写不敏感和短路径）。"""
    try:
        return os.path.samefile(path_a, path_b)
    except OSError:
        return os.path.normcase(os.path.abspath(path_a)) == os.path.normcase(os.path.abspath(path_b))


def verify_checksum(data, offset: int) -> bool:
    if offset + 7 >= len(data):
        return False
    checksum = sum(data[offset + 2:offset + 7]) & 0xFF
    return (~checksum & 0xFF) == data[offset + 7]


def parse_frames(data) -> Dict[int, int]:
    """按 FF FF 帧头 + 校验和扫描，返回 {编码器ID: 原始值}。"""
    values = {}
    index = 0
    limit = len(data) - 8
    while index <= limit:
        if data[index] != 0xFF or data[index + 1] != 0xFF:
            index += 1
            continue
        if not verify_checksum(data, index):
            index += 1
            continue
        sid = data[index + 2]
        raw = (data[index + 6] << 8) | data[index + 5]
        values[sid] = raw
        index += 8
    return values


# ===================== 串口 =====================

class StsSerialBus:
    """STS 编码器串口：发送读取命令，解析一帧原始值。"""

    def __init__(self):
        self.ser = None
        self.io_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def open(self, port: str, baudrate: int):
        if serial is None:
            raise RuntimeError(f"缺少 pyserial，请先安装：pip install pyserial ({SERIAL_IMPORT_ERROR})")
        self.close()
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            timeout=0.02,
        )

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def read_frames(self, timeout: float = 0.25) -> Dict[int, int]:
        if not self.connected:
            return {}
        with self.io_lock:
            self.ser.reset_input_buffer()
            self.ser.write(bytes(READ_COMMAND))
            data = bytearray()
            start = time.time()
            last_data = start
            while time.time() - start < timeout:
                waiting = self.ser.in_waiting
                if waiting:
                    data.extend(self.ser.read(waiting))
                    last_data = time.time()
                    if len(data) >= ENCODER_FRAME_COUNT * 8:
                        break
                elif data and time.time() - last_data > 0.005:
                    break
                else:
                    time.sleep(0.001)
        return parse_frames(data)


# ===================== 界面 =====================

class CalibrationApp(tk.Tk):
    COLUMNS = ("关节", "ID", "当前原始值", "当前角度", "正限位(°)", "负限位(°)", "原点", "操作", "状态")

    def __init__(self):
        super().__init__()
        self.title("骨架舵机原点标定")
        self.geometry("1280x780")
        self.minsize(1060, 640)

        self.joints = [JointCalibration(spec) for spec in load_joint_specs()]
        self.samples = {joint.sid: deque(maxlen=SMOOTH_WINDOW) for joint in self.joints}
        self.path_samples = {joint.sid: deque(maxlen=SMOOTH_WINDOW) for joint in self.joints}
        self.latest_raw: Dict[int, int] = {}
        self.last_raw: Dict[int, int] = {}
        self.cumulative: Dict[int, float] = {}
        self.row_ui: List[dict] = []

        self.message_queue = queue.Queue()
        self.bus = StsSerialBus()
        self.reader_thread = None
        self.stop_event = threading.Event()

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="1000000")
        self.status_var = tk.StringVar(value="未连接")
        self.summary_var = tk.StringVar(value="")

        self._configure_style()
        self._build_ui()
        self._refresh_ports()
        self._refresh_all_rows()
        self._poll_queue()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- 界面构建 ----------
    def _configure_style(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TButton", font=("Microsoft YaHei UI", 9), padding=(8, 4))
        style.configure("TLabel", font=("Microsoft YaHei UI", 9))
        style.configure("Header.TLabel", font=("Microsoft YaHei UI", 9, "bold"))
        self.configure(bg="#f4f5f7")
        style.configure("TFrame", background="#f4f5f7")

    def _build_ui(self):
        top = ttk.Frame(self, padding=(12, 10, 12, 4))
        top.pack(fill=tk.X)
        ttk.Label(top, text="串口").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=16)
        self.port_combo.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Button(top, text="刷新", command=self._refresh_ports).pack(side=tk.LEFT)
        ttk.Label(top, text="波特率").pack(side=tk.LEFT, padx=(16, 4))
        self.baud_entry = ttk.Entry(top, textvariable=self.baud_var, width=10)
        self.baud_entry.pack(side=tk.LEFT)
        self.connect_button = ttk.Button(top, text="连接", command=self._connect)
        self.connect_button.pack(side=tk.LEFT, padx=(16, 4))
        self.disconnect_button = ttk.Button(top, text="断开", command=self._disconnect, state=tk.DISABLED)
        self.disconnect_button.pack(side=tk.LEFT)
        ttk.Button(top, text="重读配置", command=self._reload_config).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(top, textvariable=self.status_var, padding=(16, 0, 0, 0)).pack(side=tk.LEFT)

        info = ttk.Frame(self, padding=(12, 0, 12, 4))
        info.pack(fill=tk.X)
        ttk.Label(
            info,
            text=f"servo_config.ini 只读使用（关节命名/direction）：{SERVO_CONFIG_PATH}",
            foreground="#68717d",
        ).pack(side=tk.LEFT)

        hint = ttk.Frame(self, padding=(12, 0, 12, 4))
        hint.pack(fill=tk.X)
        ttk.Label(
            hint,
            text="记录时请让关节从一个限位连续移动到另一个限位；非对称关节（四轴 -145°~+60°）"
                 "按额定范围计算原点，记录后限位显示为关节空间角度（支持超过 180°）。",
            foreground="#68717d",
        ).pack(side=tk.LEFT)

        table = ttk.Frame(self, padding=(12, 8))
        table.pack(fill=tk.BOTH, expand=True)
        widths = (10, 5, 10, 9, 11, 11, 8, 23, 12)
        for col, text in enumerate(self.COLUMNS):
            ttk.Label(table, text=text, style="Header.TLabel").grid(
                row=0, column=col, sticky="w", padx=6, pady=(2, 6)
            )
            table.columnconfigure(col, minsize=widths[col] * 7)

        for index, joint in enumerate(self.joints):
            row = index + 1
            ui = {}
            name_label = ttk.Label(table, text=joint.label)
            name_label.grid(row=row, column=0, sticky="w", padx=6, pady=3)
            ui["name"] = name_label
            ttk.Label(table, text=str(joint.sid)).grid(row=row, column=1, sticky="w", padx=6, pady=3)

            ui["raw"] = tk.StringVar(value="--")
            ttk.Label(table, textvariable=ui["raw"]).grid(row=row, column=2, sticky="w", padx=6, pady=3)
            ui["deg"] = tk.StringVar(value="--")
            ttk.Label(table, textvariable=ui["deg"]).grid(row=row, column=3, sticky="w", padx=6, pady=3)

            ui["pos"] = tk.StringVar(value="--")
            ttk.Label(table, textvariable=ui["pos"]).grid(row=row, column=4, sticky="w", padx=6, pady=3)
            ui["neg"] = tk.StringVar(value="--")
            ttk.Label(table, textvariable=ui["neg"]).grid(row=row, column=5, sticky="w", padx=6, pady=3)
            ui["origin"] = tk.StringVar(value="--")
            ui["origin_label"] = ttk.Label(table, textvariable=ui["origin"])
            ui["origin_label"].grid(row=row, column=6, sticky="w", padx=6, pady=3)

            actions = ttk.Frame(table)
            actions.grid(row=row, column=7, sticky="w", padx=6, pady=3)
            button_pos = ttk.Button(actions, text="记正限位", width=10,
                                    command=lambda i=index: self._record_limit(i, "pos"))
            button_pos.pack(side=tk.LEFT, padx=(0, 4))
            button_neg = ttk.Button(actions, text="记负限位", width=10,
                                    command=lambda i=index: self._record_limit(i, "neg"))
            button_neg.pack(side=tk.LEFT, padx=(0, 4))
            button_confirm = ttk.Button(actions, text="确认", width=8,
                                        command=lambda i=index: self._toggle_confirm(i))
            button_confirm.pack(side=tk.LEFT)
            ui["buttons"] = (button_pos, button_neg, button_confirm)

            ui["status"] = tk.StringVar(value="待测正限位")
            ttk.Label(table, textvariable=ui["status"]).grid(row=row, column=8, sticky="w", padx=6, pady=3)

            self.row_ui.append(ui)

        bottom = ttk.Frame(self, padding=(12, 4, 12, 12))
        bottom.pack(fill=tk.X)
        ttk.Label(bottom, textvariable=self.summary_var).pack(side=tk.LEFT)
        ttk.Button(bottom, text="导出结果...", command=self._export).pack(side=tk.RIGHT)
        ttk.Button(bottom, text="全部清除", command=self._clear_all).pack(side=tk.RIGHT, padx=(0, 8))

    # ---------- 串口 ----------
    def _refresh_ports(self):
        if serial is None:
            self.status_var.set(f"缺少 pyserial：{SERIAL_IMPORT_ERROR}")
            return
        ports = [port.device for port in serial.tools.list_ports.comports()]
        current = self.port_var.get().strip()
        if current and current not in ports:
            ports.insert(0, current)
        self.port_combo["values"] = ports
        if not current and ports:
            self.port_var.set(ports[0])

    def _reload_config(self):
        """重新读取 servo_config.ini / teleop_config.ini，并按新配置重算已记录的原点。"""
        specs = load_joint_specs()
        if [(s.label, s.sid) for s in specs] != [(j.spec.label, j.spec.sid) for j in self.joints]:
            messagebox.showwarning(
                "编码器映射已变化",
                "编码器 ID 映射与启动时不同，请重启标定程序。",
                parent=self,
            )
            return
        for index, (joint, spec) in enumerate(zip(self.joints, specs)):
            joint.spec = spec
            self._recompute(joint)
            self._refresh_row(index)
        self._update_summary()
        self.status_var.set("已重读配置（direction / 关节范围）")

    def _connect(self):
        if self.bus.connected:
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("未选择串口", "请先选择编码器串口。", parent=self)
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("参数错误", "波特率必须是数字。", parent=self)
            return
        try:
            self.bus.open(port, baud)
        except Exception as exc:
            messagebox.showerror("连接失败", f"打开串口失败：{exc}", parent=self)
            return

        self.stop_event = threading.Event()
        self._reset_tracking()
        while True:
            try:
                self.message_queue.get_nowait()
            except queue.Empty:
                break
        for joint in self.joints:
            if not joint.confirmed and (
                joint.pos_limit is not None or joint.neg_limit is not None
            ):
                # 重新连接后累计路径会重新开始，未确认的旧记录作废
                joint.pos_limit = joint.neg_limit = None
                joint.pos_path = joint.neg_path = None
                self._recompute(joint)
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.status_var.set(f"已连接 {port} @ {baud}")
        self.connect_button.configure(state=tk.DISABLED)
        self.disconnect_button.configure(state=tk.NORMAL)
        self.port_combo.configure(state=tk.DISABLED)
        self.baud_entry.configure(state=tk.DISABLED)
        self._refresh_all_rows()

    def _reset_tracking(self):
        """清空相邻帧跟踪状态和累计路径。"""
        self.latest_raw.clear()
        self.last_raw.clear()
        self.cumulative.clear()
        for samples in self.samples.values():
            samples.clear()
        for samples in self.path_samples.values():
            samples.clear()

    def _disconnect(self):
        self.stop_event.set()
        thread = self.reader_thread
        if thread is not None and thread.is_alive() and threading.current_thread() is not thread:
            thread.join(timeout=0.8)
        self.reader_thread = None
        self.bus.close()
        self.status_var.set("未连接")
        self.connect_button.configure(state=tk.NORMAL)
        self.disconnect_button.configure(state=tk.DISABLED)
        self.port_combo.configure(state=tk.NORMAL)
        self.baud_entry.configure(state=tk.NORMAL)
        self._refresh_all_rows()

    def _reader_loop(self):
        period = 1.0 / READ_HZ
        stop_event = self.stop_event
        try:
            while not stop_event.is_set():
                started = time.time()
                frames = self.bus.read_frames()
                if frames:
                    self.message_queue.put(("frame", frames))
                elapsed = time.time() - started
                if elapsed < period:
                    time.sleep(period - elapsed)
        except Exception as exc:
            self.message_queue.put(("error", str(exc)))
        finally:
            self.message_queue.put(("stopped", None))

    def _poll_queue(self):
        while True:
            try:
                kind, payload = self.message_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "frame":
                self._apply_frame(payload)
            elif kind == "error":
                self._handle_reader_error(payload)
        self.after(100, self._poll_queue)

    def _handle_reader_error(self, message):
        self._disconnect()
        self.status_var.set("串口异常，已断开")
        messagebox.showerror("串口错误", f"读取编码器失败：{message}", parent=self)

    def _apply_frame(self, frame: Dict[int, int]):
        for sid, raw in frame.items():
            self.latest_raw[sid] = raw
            if sid in self.samples:
                self.samples[sid].append(raw)
            previous = self.last_raw.get(sid)
            if previous is None:
                cumulative = float(raw)
            else:
                cumulative = self.cumulative.get(sid, float(previous)) + unwrap_delta(raw, previous)
            self.last_raw[sid] = raw
            self.cumulative[sid] = cumulative
            if sid in self.path_samples:
                self.path_samples[sid].append(cumulative)
        for index, joint in enumerate(self.joints):
            raw = self.latest_raw.get(joint.sid)
            ui = self.row_ui[index]
            if raw is None:
                ui["raw"].set("--")
                ui["deg"].set("--")
            else:
                ui["raw"].set(str(raw))
                ui["deg"].set(f"{counts_to_deg(raw):.1f}°")

    # ---------- 标定操作 ----------
    def _record_limit(self, index: int, kind: str):
        joint = self.joints[index]
        if joint.confirmed:
            return

        # 同一个按钮再点一次 = 清空该限位，便于重新测量
        if kind == "pos" and joint.pos_limit is not None:
            joint.pos_limit = None
            joint.pos_path = None
            self._recompute(joint)
            self._refresh_row(index)
            self._update_summary()
            return
        if kind == "neg" and joint.neg_limit is not None:
            joint.neg_limit = None
            joint.neg_path = None
            self._recompute(joint)
            self._refresh_row(index)
            self._update_summary()
            return

        samples = self.samples.get(joint.sid)
        path_samples = self.path_samples.get(joint.sid)
        if (
            not samples
            or len(samples) < MIN_SMOOTH_SAMPLES
            or not path_samples
            or len(path_samples) < MIN_SMOOTH_SAMPLES
        ):
            messagebox.showwarning(
                "读数不足",
                "还没有收到足够的编码器数据，请确认已连接并等待片刻。",
                parent=self,
            )
            return
        spread = max(path_samples) - min(path_samples)
        if spread > STABLE_TOL_COUNTS:
            messagebox.showwarning(
                "读数还在变化",
                f"{joint.label} 的读数还在变化（最近波动约 {counts_to_deg(spread):.1f}°），"
                "请等关节停稳后再记录。",
                parent=self,
            )
            return

        path_value = sum(path_samples) / len(path_samples)
        raw_value = circular_mean(samples)
        if kind == "pos":
            joint.pos_limit = raw_value
            joint.pos_path = path_value
        else:
            joint.neg_limit = raw_value
            joint.neg_path = path_value
        self._recompute(joint)
        if joint.travel is not None:
            if (
                joint.rated_travel_deg is not None
                and joint.direction_mismatch
            ):
                messagebox.showwarning(
                    "方向与记录顺序不一致",
                    f"{joint.label}：正/负限位的记录顺序与 servo_config.ini 中 "
                    f"direction={joint.spec.direction} 不一致。\n"
                    "原点按 direction 计算、不受记录顺序影响；但如果正限位确实"
                    "在关节正方向，则 direction 可能配反了（四轴原点会偏约 85°）。\n"
                    "请核对后再确认。",
                    parent=self,
                )
            elif joint.travel < MIN_TRAVEL_COUNTS:
                messagebox.showwarning(
                    "行程过小",
                    f"{joint.label} 的正负限位几乎相同"
                    f"（{counts_to_deg(joint.travel):.2f}°），请重新测量。",
                    parent=self,
                )
            elif joint.travel > TRAVEL_LIMIT_COUNTS:
                messagebox.showwarning(
                    "行程异常",
                    f"{joint.label} 的记录行程超过一圈"
                    f"（{counts_to_deg(joint.travel):.1f}°），请检查是否绕圈或记录有误。",
                    parent=self,
                )
            elif (
                joint.rated_travel_error_deg is not None
                and abs(joint.rated_travel_error_deg) > RANGE_TOL_DEG
            ):
                messagebox.showwarning(
                    "行程与额定范围不符",
                    f"{joint.label} 实测行程 {joint.travel_deg:.1f}°，"
                    f"额定范围 {joint.spec.min_deg:.0f}°~{joint.spec.max_deg:+.0f}°"
                    f"（{joint.rated_travel_deg:.0f}°）。\n"
                    "请确认正、负限位都确实转到了机械限位。",
                    parent=self,
                )
            if joint.spec.direction not in (1, -1) and joint.rated_travel_deg is not None:
                messagebox.showwarning(
                    "direction 未配置",
                    f"servo_config.ini 未找到 ID{joint.sid} 的 direction，"
                    f"已按本次记录推断为 {joint.direction_inferred}。\n"
                    "合并零点时请一并把 direction 设为该值。",
                    parent=self,
                )
        self._refresh_row(index)
        self._update_summary()

    @staticmethod
    def _recompute(joint: JointCalibration):
        """按实际运动路径和关节额定范围计算原点。

        - 路径中点用展开后的累计值，可正确处理 >180° 的行程；
        - 对称关节（未配置范围）：原点 = 中点；
        - 非对称关节（如四轴 -145°~+60°）：把零点从中点偏移额定范围中点，
          使正/负限位分别落在 max_deg / min_deg；
        - 同时用 direction 校验记录的正负限位是否记反。
        """
        spec = joint.spec
        joint.rated_travel_deg = spec.rated_travel_deg
        joint.direction_inferred = None
        joint.direction_mismatch = False

        if joint.pos_path is None or joint.neg_path is None:
            joint.origin = None
            joint.travel = None
            joint.travel_deg = None
            joint.direction_used = spec.direction if spec.direction in (1, -1) else 1
            return

        scale = 360.0 / ENCODER_RESOLUTION
        delta_counts = joint.pos_path - joint.neg_path      # 负限位 → 正限位的编码器增量
        inferred = 1 if delta_counts > 0 else -1
        direction = spec.direction if spec.direction in (1, -1) else inferred
        joint.direction_used = direction
        joint.direction_inferred = inferred
        joint.direction_mismatch = spec.direction in (1, -1) and direction != inferred
        joint.travel_deg = direction * delta_counts * scale  # 负限位→正限位，应为正
        joint.travel = int(round(abs(delta_counts)))

        mid_path = (joint.pos_path + joint.neg_path) / 2.0
        if spec.min_deg is not None and spec.max_deg is not None:
            mid_deg = (spec.min_deg + spec.max_deg) / 2.0
            joint.origin = (mid_path - direction * mid_deg / scale) % ENCODER_RESOLUTION
        else:
            joint.origin = mid_path % ENCODER_RESOLUTION

    def _toggle_confirm(self, index: int):
        joint = self.joints[index]
        if not joint.confirmed:
            if joint.origin is None:
                messagebox.showwarning(
                    "无法确认",
                    f"请先记录 {joint.label} 的正限位和负限位。",
                    parent=self,
                )
                return
            if joint.rated_travel_deg is not None and joint.direction_mismatch:
                if not messagebox.askyesno(
                    "方向与记录顺序不一致",
                    f"{joint.label}：记录顺序与 direction={joint.spec.direction} 不一致。\n"
                    "原点按 direction 计算；若 direction 配反，四轴原点会偏约 85°。\n"
                    "仍要确认吗？",
                    parent=self,
                ):
                    return
            if (
                joint.rated_travel_error_deg is not None
                and abs(joint.rated_travel_error_deg) > RANGE_TOL_DEG
            ):
                if not messagebox.askyesno(
                    "行程与额定范围不符",
                    f"{joint.label} 实测行程 {joint.travel_deg:.1f}°，"
                    f"额定 {joint.rated_travel_deg:.0f}°。\n"
                    "两端是否都确实转到了机械限位？仍要确认吗？",
                    parent=self,
                ):
                    return
        joint.confirmed = not joint.confirmed
        self._refresh_row(index)
        self._update_summary()

    def _clear_all(self):
        if not messagebox.askyesno("全部清除", "确定清除所有关节的限位记录和确认状态？", parent=self):
            return
        for joint in self.joints:
            joint.pos_limit = None
            joint.neg_limit = None
            joint.pos_path = None
            joint.neg_path = None
            joint.confirmed = False
            self._recompute(joint)
        self._refresh_all_rows()
        self._update_summary()

    # ---------- 行刷新 ----------
    def _refresh_row(self, index: int):
        joint = self.joints[index]
        ui = self.row_ui[index]
        # 原点已知后按关节空间角度显示（套用 direction），便于和额定范围对照
        if joint.origin is None:
            fmt_limit = lambda value: "--" if value is None else f"{counts_to_deg(value):.1f}°"
        else:
            direction = joint.direction_used
            fmt_limit = lambda value: (
                "--" if value is None else f"{joint_deg(value, joint.origin, direction):+.1f}°"
            )
        ui["pos"].set(fmt_limit(joint.pos_limit))
        ui["neg"].set(fmt_limit(joint.neg_limit))
        ui["origin"].set("--" if joint.origin is None else f"{joint.origin:.0f}")

        abnormal = (
            joint.rated_travel_deg is not None
            and (
                joint.direction_mismatch
                or (
                    joint.rated_travel_error_deg is not None
                    and abs(joint.rated_travel_error_deg) > RANGE_TOL_DEG
                )
            )
        )
        ui["origin_label"].configure(foreground="#c0392b" if abnormal else "#20252d")

        button_pos, button_neg, button_confirm = ui["buttons"]
        can_record = self.bus.connected and not joint.confirmed
        record_state = tk.NORMAL if can_record else tk.DISABLED
        button_pos.configure(
            text="清空正限位" if joint.pos_limit is not None else "记正限位",
            state=record_state,
        )
        button_neg.configure(
            text="清空负限位" if joint.neg_limit is not None else "记负限位",
            state=record_state,
        )

        # 关节名称颜色：已记一个限位=蓝色，正负限位都记录=绿色
        if joint.pos_limit is not None and joint.neg_limit is not None:
            name_color = "#1f8b4c"
        elif joint.pos_limit is not None or joint.neg_limit is not None:
            name_color = "#1c6fd8"
        else:
            name_color = "#20252d"
        ui["name"].configure(foreground=name_color)

        button_confirm.configure(text="取消确认" if joint.confirmed else "确认")
        if joint.confirmed:
            button_confirm.configure(state=tk.NORMAL)
        else:
            button_confirm.configure(state=tk.NORMAL if joint.origin is not None else tk.DISABLED)

        if joint.confirmed:
            status = "已确认 ✓"
        elif joint.origin is not None:
            status = "待确认"
        elif joint.pos_limit is None:
            status = "待测正限位"
        else:
            status = "待测负限位"
        ui["status"].set(status)

    def _refresh_all_rows(self):
        for index in range(len(self.joints)):
            self._refresh_row(index)
        self._update_summary()

    def _update_summary(self):
        confirmed = sum(1 for joint in self.joints if joint.confirmed)
        total = len(self.joints)
        if confirmed == total:
            self.summary_var.set(f"已确认 {confirmed}/{total}  全部完成，可以导出结果")
        else:
            self.summary_var.set(f"已确认 {confirmed}/{total}  未确认关节不会导出")

    # ---------- 导出 ----------
    def _export(self):
        confirmed = [joint for joint in self.joints if joint.confirmed]
        if not confirmed:
            messagebox.showerror("无法导出", "还没有已确认的关节。", parent=self)
            return
        unconfirmed = len(self.joints) - len(confirmed)
        if unconfirmed and not messagebox.askyesno(
            "导出确认",
            f"还有 {unconfirmed} 个关节未确认，只导出已确认的 {len(confirmed)} 个？",
            parent=self,
        ):
            return

        path = filedialog.asksaveasfilename(
            parent=self,
            title="导出标定结果",
            initialdir=SCRIPT_DIR,
            initialfile=f"origin_calibration_{time.strftime('%Y%m%d_%H%M%S')}.ini",
            defaultextension=".ini",
            filetypes=[("INI 文件", "*.ini"), ("所有文件", "*.*")],
        )
        if not path:
            return
        if is_same_file(path, SERVO_CONFIG_PATH):
            messagebox.showerror(
                "禁止覆盖",
                "servo_config.ini 只能读取，禁止写入。请选择其他保存位置。",
                parent=self,
            )
            return

        lines = [
            "# 骨架舵机原点标定结果",
            f"# 生成时间: {time.strftime('%Y-%m-%d %H:%M:%S')}",
            "# zero_offset_raw 为编码器原始值（0~4095）",
            "# 角度为关节空间角度（已按 direction 换算），限位应落在关节额定范围内",
            "# 注意: 本文件不是 servo_config.ini，请人工核对后再合并",
            "",
        ]
        for joint in confirmed:
            origin_int = int(round(joint.origin)) % ENCODER_RESOLUTION
            direction = joint.direction_used
            pos_disp = joint_deg(joint.pos_limit, joint.origin, direction)
            neg_disp = joint_deg(joint.neg_limit, joint.origin, direction)
            pos_raw = int(round(joint.pos_limit)) % ENCODER_RESOLUTION
            neg_raw = int(round(joint.neg_limit)) % ENCODER_RESOLUTION
            extra = ""
            if joint.rated_travel_deg is not None:
                extra = (
                    f" 额定范围={joint.spec.min_deg:.0f}~{joint.spec.max_deg:+.0f}°"
                    f" 实测行程={joint.travel_deg:.1f}°"
                )
            lines.append(f"[{joint.sid}]")
            lines.append(f"zero_offset_raw = {origin_int}")
            lines.append(
                f"# 关节={joint.label} "
                f"正限位={pos_raw}({pos_disp:+.1f}°) "
                f"负限位={neg_raw}({neg_disp:+.1f}°) "
                f"direction={direction}{extra}"
            )
            lines.append("")

        try:
            with open(path, "w", encoding="utf-8") as handle:
                handle.write("\n".join(lines))
        except Exception as exc:
            messagebox.showerror("导出失败", str(exc), parent=self)
            return
        messagebox.showinfo("导出完成", f"已导出 {len(confirmed)} 个关节：\n{path}", parent=self)

    # ---------- 关闭 ----------
    def _on_close(self):
        if self.bus.connected:
            self._disconnect()
        self.destroy()


def main():
    app = CalibrationApp()
    app.mainloop()


if __name__ == "__main__":
    main()
