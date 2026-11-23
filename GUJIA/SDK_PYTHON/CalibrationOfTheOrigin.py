# -*- coding: utf-8 -*-
"""
骨架舵机原点标定程序

功能：
- 通过串口读取 16 路 STS 舵机编码器的原始值（0~4095）
- 操作人员把每个关节分别转到机械正限位、负限位，程序逐个记录两端原始值
- 自动计算原点（正负限位之间的中点），由操作人员在前端逐个确认
- 标定结果导出为独立的 .ini 文件

重要：servo_config.ini 只读取 [encoder_map] 用于关节命名，本程序
      不会写入或修改 servo_config.ini。

与摇操程序（gujiayaocao_0817.py）完全独立，可单独运行。
"""
import configparser
import math
import os
import queue
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


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SERVO_CONFIG_PATH = os.path.join(SCRIPT_DIR, "servo_config.ini")

ENCODER_RESOLUTION = 4096
ENCODER_FRAME_COUNT = 16
READ_HZ = 50.0
SMOOTH_WINDOW = 12          # 记录限位时用于平滑的采样点数
MIN_SMOOTH_SAMPLES = 5      # 至少收到多少个点才允许记录
MIN_TRAVEL_COUNTS = 8       # 行程过小视为异常（约 0.7°）

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


@dataclass
class JointSpec:
    label: str
    sid: int


@dataclass
class JointCalibration:
    spec: JointSpec
    pos_limit: Optional[float] = None
    neg_limit: Optional[float] = None
    origin: Optional[float] = None
    travel: Optional[int] = None
    confirmed: bool = False

    @property
    def sid(self) -> int:
        return self.spec.sid

    @property
    def label(self) -> str:
        return self.spec.label


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


def load_joint_specs() -> List[JointSpec]:
    """读取 servo_config.ini 的 [encoder_map]；文件不存在或格式不对时用默认映射。"""
    arm_a = list(DEFAULT_ARM_A_IDS)
    arm_b = list(DEFAULT_ARM_B_IDS)
    left_gripper = DEFAULT_LEFT_GRIPPER_ID
    right_gripper = DEFAULT_RIGHT_GRIPPER_ID

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

    specs = [JointSpec(f"A{i + 1}", sid) for i, sid in enumerate(arm_a)]
    specs.append(JointSpec("左夹爪", left_gripper))
    specs.extend(JointSpec(f"B{i + 1}", sid) for i, sid in enumerate(arm_b))
    specs.append(JointSpec("右夹爪", right_gripper))
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


def compute_origin(pos_counts: float, neg_counts: float):
    """由正、负限位原始值计算原点（较短弧的中点）与行程。"""
    delta = (neg_counts - pos_counts) % ENCODER_RESOLUTION
    if delta <= ENCODER_RESOLUTION // 2:
        travel = delta
        origin = pos_counts + delta / 2.0
    else:
        travel = ENCODER_RESOLUTION - delta
        origin = pos_counts - travel / 2.0
    return origin % ENCODER_RESOLUTION, travel


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
    COLUMNS = ("关节", "ID", "当前原始值", "当前角度", "正限位", "负限位", "行程", "原点", "操作", "状态")

    def __init__(self):
        super().__init__()
        self.title("骨架舵机原点标定")
        self.geometry("1280x780")
        self.minsize(1060, 640)

        self.joints = [JointCalibration(spec) for spec in load_joint_specs()]
        self.samples = {joint.sid: deque(maxlen=SMOOTH_WINDOW) for joint in self.joints}
        self.latest_raw: Dict[int, int] = {}
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
        ttk.Label(top, textvariable=self.status_var, padding=(16, 0, 0, 0)).pack(side=tk.LEFT)

        info = ttk.Frame(self, padding=(12, 0, 12, 4))
        info.pack(fill=tk.X)
        ttk.Label(
            info,
            text=f"servo_config.ini 只读使用（关节命名）：{SERVO_CONFIG_PATH}",
            foreground="#68717d",
        ).pack(side=tk.LEFT)

        table = ttk.Frame(self, padding=(12, 8))
        table.pack(fill=tk.BOTH, expand=True)
        widths = (10, 5, 10, 9, 8, 8, 9, 8, 23, 12)
        for col, text in enumerate(self.COLUMNS):
            ttk.Label(table, text=text, style="Header.TLabel").grid(
                row=0, column=col, sticky="w", padx=6, pady=(2, 6)
            )
            table.columnconfigure(col, minsize=widths[col] * 7)

        for index, joint in enumerate(self.joints):
            row = index + 1
            ui = {}
            ttk.Label(table, text=joint.label).grid(row=row, column=0, sticky="w", padx=6, pady=3)
            ttk.Label(table, text=str(joint.sid)).grid(row=row, column=1, sticky="w", padx=6, pady=3)

            ui["raw"] = tk.StringVar(value="--")
            ttk.Label(table, textvariable=ui["raw"]).grid(row=row, column=2, sticky="w", padx=6, pady=3)
            ui["deg"] = tk.StringVar(value="--")
            ttk.Label(table, textvariable=ui["deg"]).grid(row=row, column=3, sticky="w", padx=6, pady=3)

            ui["pos"] = tk.StringVar(value="--")
            ttk.Label(table, textvariable=ui["pos"]).grid(row=row, column=4, sticky="w", padx=6, pady=3)
            ui["neg"] = tk.StringVar(value="--")
            ttk.Label(table, textvariable=ui["neg"]).grid(row=row, column=5, sticky="w", padx=6, pady=3)
            ui["travel"] = tk.StringVar(value="--")
            ttk.Label(table, textvariable=ui["travel"]).grid(row=row, column=6, sticky="w", padx=6, pady=3)
            ui["origin"] = tk.StringVar(value="--")
            ttk.Label(table, textvariable=ui["origin"]).grid(row=row, column=7, sticky="w", padx=6, pady=3)

            actions = ttk.Frame(table)
            actions.grid(row=row, column=8, sticky="w", padx=6, pady=3)
            button_pos = ttk.Button(actions, text="记正限位", width=9,
                                    command=lambda i=index: self._record_limit(i, "pos"))
            button_pos.pack(side=tk.LEFT, padx=(0, 4))
            button_neg = ttk.Button(actions, text="记负限位", width=9,
                                    command=lambda i=index: self._record_limit(i, "neg"))
            button_neg.pack(side=tk.LEFT, padx=(0, 4))
            button_confirm = ttk.Button(actions, text="确认", width=8,
                                        command=lambda i=index: self._toggle_confirm(i))
            button_confirm.pack(side=tk.LEFT)
            ui["buttons"] = (button_pos, button_neg, button_confirm)

            ui["status"] = tk.StringVar(value="待测正限位")
            ttk.Label(table, textvariable=ui["status"]).grid(row=row, column=9, sticky="w", padx=6, pady=3)

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
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.status_var.set(f"已连接 {port} @ {baud}")
        self.connect_button.configure(state=tk.DISABLED)
        self.disconnect_button.configure(state=tk.NORMAL)
        self.port_combo.configure(state=tk.DISABLED)
        self.baud_entry.configure(state=tk.DISABLED)
        self._refresh_all_rows()

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
        samples = self.samples.get(joint.sid)
        if not samples or len(samples) < MIN_SMOOTH_SAMPLES:
            messagebox.showwarning(
                "读数不足",
                "还没有收到足够的编码器数据，请确认已连接并等待片刻。",
                parent=self,
            )
            return
        value = circular_mean(samples)
        if kind == "pos":
            joint.pos_limit = value
        else:
            joint.neg_limit = value

        if joint.pos_limit is not None and joint.neg_limit is not None:
            joint.origin, joint.travel = compute_origin(joint.pos_limit, joint.neg_limit)
            if joint.travel < MIN_TRAVEL_COUNTS:
                messagebox.showwarning(
                    "行程过小",
                    f"{joint.label} 的正负限位几乎相同"
                    f"（{counts_to_deg(joint.travel):.2f}°），请重新测量。",
                    parent=self,
                )
        self._refresh_row(index)
        self._update_summary()

    def _toggle_confirm(self, index: int):
        joint = self.joints[index]
        if not joint.confirmed and joint.origin is None:
            messagebox.showwarning(
                "无法确认",
                f"请先记录 {joint.label} 的正限位和负限位。",
                parent=self,
            )
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
            joint.origin = None
            joint.travel = None
            joint.confirmed = False
        self._refresh_all_rows()
        self._update_summary()

    # ---------- 行刷新 ----------
    def _refresh_row(self, index: int):
        joint = self.joints[index]
        ui = self.row_ui[index]
        fmt_raw = lambda value: "--" if value is None else f"{value:.0f}"
        ui["pos"].set(fmt_raw(joint.pos_limit))
        ui["neg"].set(fmt_raw(joint.neg_limit))
        if joint.origin is not None and joint.travel is not None:
            ui["origin"].set(f"{joint.origin:.0f}")
            ui["travel"].set(f"{counts_to_deg(joint.travel):.1f}°")
        else:
            ui["origin"].set("--")
            ui["travel"].set("--")

        button_pos, button_neg, button_confirm = ui["buttons"]
        can_record = self.bus.connected and not joint.confirmed
        record_state = tk.NORMAL if can_record else tk.DISABLED
        button_pos.configure(state=record_state)
        button_neg.configure(state=record_state)

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
            "# 注意: 本文件不是 servo_config.ini，请人工核对后再合并",
            "",
        ]
        for joint in confirmed:
            origin_int = int(round(joint.origin)) % ENCODER_RESOLUTION
            lines.append(f"[{joint.sid}]")
            lines.append(f"zero_offset_raw = {origin_int}")
            lines.append(
                f"# 关节={joint.label} 正限位={joint.pos_limit:.0f} "
                f"负限位={joint.neg_limit:.0f} 行程={counts_to_deg(joint.travel):.1f}°"
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
