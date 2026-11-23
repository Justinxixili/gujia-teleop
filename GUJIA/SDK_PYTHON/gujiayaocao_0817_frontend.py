# -*- coding: utf-8 -*-
"""
骨架摇操图形控制台

这个界面作为 gujiayaocao_0817.py 的启动和控制面板：
- 启动/停止原控制脚本
- 配置机器人 IP、编码器串口和波特率
- 通过按钮触发原脚本已经支持的 s/d/ESC 控制
- 实时显示关节对齐状态和相机拼接画面
"""
import json
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    from pynput.keyboard import Controller, Key
except Exception:
    Controller = None
    Key = None


FROZEN = bool(getattr(sys, "frozen", False))
if FROZEN:
    # 打包成 exe 后，配置和输出都放在 exe 所在目录
    SCRIPT_DIR = os.path.dirname(os.path.abspath(sys.executable))
else:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_SCRIPT = os.path.join(SCRIPT_DIR, "gujiayaocao_0817.py")


CAMERA_ROLES = (
    ("head", "头部 D435"),
    ("left_wrist", "左腕 D405"),
    ("right_wrist", "右腕 D405"),
)
CAMERA_DEFAULT_DIR = os.path.join(SCRIPT_DIR, "camera_dataset")
CAMERA_FIRST_FRAME_TIMEOUT_MS = 1500
CAMERA_PREVIEW_MIN_HEIGHT = 620


class RealSenseCameraHub:
    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self.rs = None
        self.np = None
        self.ctx = None
        self.pipelines = {}
        self.latest_frames = {}
        self.assignments = {}

    def start(self):
        import pyrealsense2 as rs
        import numpy as np

        self.rs = rs
        self.np = np
        self.ctx = rs.context()
        try:
            devices = [self._device_info(device) for device in self.ctx.query_devices()]
        except Exception as exc:
            raise RuntimeError(f"RealSense device access failed: {exc}") from exc
        if not devices:
            raise RuntimeError("未发现 RealSense 相机")

        self.assignments = self._assign_roles(devices)
        if not self.assignments:
            raise RuntimeError("未能分配 RealSense 相机")

        started = {}
        try:
            for role, device in self.assignments.items():
                pipeline, size, color_format, first_frame = self._start_pipeline(device)
                started[role] = {
                    "pipeline": pipeline,
                    "serial": device["serial"],
                    "name": device["name"],
                    "size": size,
                    "format": color_format,
                }
                self.latest_frames[role] = first_frame
            self.pipelines = started
            return dict(started)
        except Exception:
            for item in started.values():
                try:
                    item["pipeline"].stop()
                except Exception:
                    pass
            self.pipelines = {}
            raise

    def stop(self):
        for item in list(self.pipelines.values()):
            try:
                item["pipeline"].stop()
            except Exception:
                pass
        self.pipelines = {}
        self.latest_frames = {}

    def read_frames(self):
        for role, item in self.pipelines.items():
            frameset = item["pipeline"].poll_for_frames()
            if not frameset:
                continue
            color_frame = frameset.get_color_frame()
            if not color_frame:
                continue
            self.latest_frames[role] = self._frame_to_rgb(color_frame, item.get("format"))
        return dict(self.latest_frames)

    def swap_wrist_roles(self):
        left = self.pipelines.get("left_wrist")
        right = self.pipelines.get("right_wrist")
        if left is None or right is None:
            return False
        self.pipelines["left_wrist"], self.pipelines["right_wrist"] = right, left
        self.assignments["left_wrist"], self.assignments["right_wrist"] = (
            self.assignments["right_wrist"],
            self.assignments["left_wrist"],
        )
        left_frame = self.latest_frames.get("left_wrist")
        right_frame = self.latest_frames.get("right_wrist")
        if left_frame is not None or right_frame is not None:
            self.latest_frames["left_wrist"] = right_frame
            self.latest_frames["right_wrist"] = left_frame
        return True

    def _device_info(self, device):
        def get_info(kind, default=""):
            try:
                return device.get_info(kind)
            except Exception:
                return default

        return {
            "serial": get_info(self.rs.camera_info.serial_number),
            "name": get_info(self.rs.camera_info.name, "RealSense"),
            "product_line": get_info(self.rs.camera_info.product_line),
        }

    def _assign_roles(self, devices):
        unused = list(devices)
        assignments = {}

        head = self._take_first(unused, "D455") or self._take_first(unused, "D435") or self._take_first(unused, "")
        if head:
            assignments["head"] = head

        left = self._take_first(unused, "D405") or self._take_first(unused, "")
        right = self._take_first(unused, "D405") or self._take_first(unused, "")
        if left:
            assignments["left_wrist"] = left
        if right:
            assignments["right_wrist"] = right
        return assignments

    def _take_first(self, devices, name_part):
        name_part = name_part.upper()
        for index, device in enumerate(devices):
            name = f"{device.get('name', '')} {device.get('product_line', '')}".upper()
            if not name_part or name_part in name:
                return devices.pop(index)
        return None

    def _start_pipeline(self, device):
        serial = device["serial"]
        last_error = None
        profiles = self._candidate_color_profiles(serial)
        seen = set()
        for width, height, fps, color_format in profiles:
            key = (width, height, fps, self._format_label(color_format))
            if key in seen:
                continue
            seen.add(key)
            pipeline = self.rs.pipeline()
            config = self.rs.config()
            config.enable_device(serial)
            config.enable_stream(self.rs.stream.color, width, height, color_format, fps)
            try:
                pipeline.start(config)
                first_frame = self._wait_first_color_frame(
                    pipeline,
                    serial,
                    CAMERA_FIRST_FRAME_TIMEOUT_MS,
                    color_format,
                )
                return pipeline, (width, height, fps), self._format_label(color_format), first_frame
            except Exception as exc:
                last_error = exc
                try:
                    pipeline.stop()
                except Exception:
                    pass
        raise RuntimeError(f"相机 {serial} 启动失败: {last_error}")

    def _candidate_color_profiles(self, serial):
        preferred = [
            (self.width, self.height, self.fps),
            (640, 480, 30),
            (848, 480, 30),
            (424, 240, 30),
            (320, 240, 30),
        ]
        format_order = {
            self._format_label(self.rs.format.rgb8): 0,
            self._format_label(self.rs.format.bgr8): 1,
            self._format_label(self.rs.format.rgba8): 2,
            self._format_label(self.rs.format.bgra8): 3,
            self._format_label(self.rs.format.y8): 4,
        }
        profiles = [
            profile
            for profile in self._iter_color_profiles(serial)
            if self._format_label(profile[3]) in format_order
        ]
        if not profiles:
            return [
                (width, height, fps, self.rs.format.rgb8)
                for width, height, fps in preferred
            ]

        def score(profile):
            width, height, fps, color_format = profile
            try:
                preferred_index = preferred.index((width, height, fps))
            except ValueError:
                preferred_index = len(preferred)
            distance = abs(width - self.width) + abs(height - self.height) + abs(fps - self.fps) * 10
            return (
                preferred_index,
                format_order.get(self._format_label(color_format), 99),
                distance,
                abs(fps - self.fps),
                width * height,
            )

        return sorted(profiles, key=score)[:4]

    def _iter_color_profiles(self, serial):
        try:
            devices = list(self.ctx.query_devices())
        except Exception:
            return
        for device in devices:
            try:
                if device.get_info(self.rs.camera_info.serial_number) != serial:
                    continue
            except Exception:
                continue
            try:
                sensors = list(device.query_sensors())
            except Exception:
                return
            for sensor in sensors:
                try:
                    stream_profiles = list(sensor.get_stream_profiles())
                except Exception:
                    continue
                for profile in stream_profiles:
                    try:
                        if profile.stream_type() != self.rs.stream.color:
                            continue
                        video_profile = profile.as_video_stream_profile()
                        yield (
                            video_profile.width(),
                            video_profile.height(),
                            profile.fps(),
                            profile.format(),
                        )
                    except Exception:
                        continue
            return

    def _format_label(self, color_format):
        return str(color_format).split(".")[-1].lower()

    def _frame_to_rgb(self, color_frame, color_format=None):
        frame = self.np.asanyarray(color_frame.get_data()).copy()
        format_name = self._format_label(color_format or color_frame.profile.format())
        if format_name == "bgr8":
            return frame[:, :, ::-1].copy()
        if format_name == "rgba8":
            return frame[:, :, :3].copy()
        if format_name == "bgra8":
            return frame[:, :, 2::-1].copy()
        if format_name == "y8" and frame.ndim == 2:
            return self.np.repeat(frame[:, :, None], 3, axis=2)
        return frame

    def _wait_first_color_frame(self, pipeline, serial, timeout_ms, color_format):
        deadline = time.time() + timeout_ms / 1000.0
        last_error = None
        while time.time() < deadline:
            try:
                frameset = pipeline.wait_for_frames(500)
            except Exception as exc:
                last_error = exc
                continue
            color_frame = frameset.get_color_frame()
            if color_frame:
                return self._frame_to_rgb(color_frame, color_format)
        details = f": {last_error}" if last_error else ""
        raise RuntimeError(f"相机 {serial} 已打开，但 {timeout_ms} ms 内没有收到彩色帧{details}")


class TeleopFrontend(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("骨架摇操控制台")
        self.geometry("1120x760")
        self.minsize(760, 620)

        self.process = None
        self.reader_thread = None
        self.log_queue = queue.Queue()
        self.keyboard = Controller() if Controller is not None else None

        self.robot_ip_var = tk.StringVar(value="192.168.1.190")
        self.port_var = tk.StringVar(value="/dev/ttyACM0")
        self.baud_var = tk.StringVar(value="1000000")
        self.status_var = tk.StringVar(value="未启动")
        self.pid_var = tk.StringVar(value="-")
        self.alignment_summary_var = tk.StringVar(value="未收到对齐数据")
        self.theme_var = tk.StringVar(value="light")
        self.camera_status_var = tk.StringVar(value="相机未启动")
        self.camera_dir_var = tk.StringVar(value=CAMERA_DEFAULT_DIR)
        self.camera_info_var = tk.StringVar(value="等待启动相机")


        self.sidebar_collapsed = False
        self.busy = False
        self.alignment_cells = {}
        self.camera_hub = None
        self.camera_thread = None
        self.camera_stop_event = threading.Event()
        self.camera_frames_lock = threading.Lock()
        self.camera_latest_frames = {}
        self.camera_latest_meta = {}
        self.camera_photo = None
        self.camera_running = False
        self.camera_capture_count = 0
        self.pil_image = None
        self.pil_image_tk = None

        self._configure_style()
        self._build_ui()
        self._apply_theme()
        self._refresh_ports()
        self._append_log("[界面] 等待启动。可点击“启动程序”。\n")
        self._poll_log_queue()
        self._refresh_state()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _palette(self):
        if self.theme_var.get() == "dark":
            return {
                "bg": "#111315",
                "surface": "#181b1f",
                "surface_alt": "#20242a",
                "border": "#323840",
                "text": "#edf1f5",
                "muted": "#9aa3ad",
                "accent": "#8aa4b8",
                "accent_bg": "#24313a",
                "danger": "#d27b75",
                "success": "#75a885",
                "input": "#15181c",
            }
        return {
            "bg": "#f4f5f7",
            "surface": "#ffffff",
            "surface_alt": "#f8fafc",
            "border": "#d9dee5",
            "text": "#20252d",
            "muted": "#68717d",
            "accent": "#5f7f8f",
            "accent_bg": "#eef4f6",
            "danger": "#a85852",
            "success": "#4f8061",
            "input": "#ffffff",
        }

    def _configure_style(self):
        self.style = ttk.Style(self)
        try:
            self.style.theme_use("clam")
        except tk.TclError:
            pass

    def _build_ui(self):
        self.shell = tk.Frame(self, bd=0, highlightthickness=0)
        self.shell.pack(fill=tk.BOTH, expand=True)
        self.shell.columnconfigure(1, weight=1)
        self.shell.rowconfigure(0, weight=1)

        self.sidebar = tk.Frame(self.shell, width=248, bd=0, highlightthickness=1)
        self.sidebar.grid(row=0, column=0, sticky="ns")
        self.sidebar.grid_propagate(False)
        self.sidebar.rowconfigure(1, weight=1)

        self.main = tk.Frame(self.shell, bd=0, highlightthickness=0)
        self.main.grid(row=0, column=1, sticky="nsew")
        self.main.columnconfigure(0, weight=1)
        self.main.columnconfigure(1, weight=0)
        self.main.rowconfigure(1, weight=1)

        self._build_sidebar()
        self._build_topbar()
        self._build_content()
        self.bind("<Configure>", self._layout_center)

    def _build_sidebar(self):
        self.sidebar_head = tk.Frame(self.sidebar, bd=0, highlightthickness=0)
        self.sidebar_head.grid(row=0, column=0, sticky="ew", padx=14, pady=(16, 10))
        self.sidebar_head.columnconfigure(1, weight=1)

        self.logo_mark = tk.Label(self.sidebar_head, text="M", width=3, height=2)
        self.logo_mark.grid(row=0, column=0, sticky="w")
        self.logo_title = tk.Label(self.sidebar_head, text="骨架摇操", anchor="w")
        self.logo_title.grid(row=0, column=1, sticky="ew", padx=(10, 6))
        self.sidebar_toggle = ttk.Button(self.sidebar_head, text="<", width=3, command=self._toggle_sidebar)
        self.sidebar_toggle.grid(row=0, column=2, sticky="e")

        self.sidebar_bottom = tk.Frame(self.sidebar, bd=0, highlightthickness=0)
        self.sidebar_bottom.grid(row=1, column=0, sticky="sew", padx=14, pady=(0, 16))
        self.sidebar_bottom.columnconfigure(0, weight=1)
        self.settings_button = ttk.Button(self.sidebar_bottom, text="设置", command=self._focus_settings)
        self.settings_button.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        self.theme_button = ttk.Button(self.sidebar_bottom, text="深色模式", command=self._toggle_theme)
        self.theme_button.grid(row=1, column=0, sticky="ew")

    def _build_topbar(self):
        self.topbar = tk.Frame(self.main, bd=0, highlightthickness=0)
        self.topbar.grid(row=0, column=0, sticky="ew")
        self.topbar.columnconfigure(0, weight=1)

        title_box = tk.Frame(self.topbar, bd=0, highlightthickness=0)
        title_box.grid(row=0, column=0, sticky="w", padx=28, pady=(18, 12))
        self.page_title = tk.Label(title_box, text="骨架摇操控制台", anchor="w")
        self.page_title.grid(row=0, column=0, sticky="w")
        self.page_subtitle = tk.Label(title_box, text="机器人启动、对齐、遥操与相机监控", anchor="w")
        self.page_subtitle.grid(row=1, column=0, sticky="w", pady=(3, 0))

        self.status_pill = tk.Label(self.topbar, textvariable=self.status_var, padx=12, pady=5)
        self.status_pill.grid(row=0, column=1, sticky="e", padx=(0, 10))
        self.pid_label = tk.Label(self.topbar, textvariable=self.pid_var, padx=10, pady=5)
        self.pid_label.grid(row=0, column=2, sticky="e", padx=(0, 8))
        ttk.Button(self.topbar, text="刷新", command=self._refresh_ports).grid(row=0, column=3, sticky="e", padx=(0, 8))
        ttk.Button(self.topbar, text="目录", command=self._open_script_dir).grid(row=0, column=4, sticky="e", padx=(0, 28))

    def _build_content(self):
        self.content_canvas = tk.Canvas(self.main, bd=0, highlightthickness=0)
        self.content_canvas.grid(row=1, column=0, sticky="nsew")
        self.content_scrollbar = ttk.Scrollbar(self.main, orient=tk.VERTICAL, command=self.content_canvas.yview)
        self.content_scrollbar.grid(row=1, column=1, sticky="ns")
        self.content_canvas.configure(yscrollcommand=self.content_scrollbar.set)

        self.center = tk.Frame(self.content_canvas, bd=0, highlightthickness=0)
        self.center_window = self.content_canvas.create_window((0, 0), window=self.center, anchor="nw")
        self.center.columnconfigure(0, weight=1)
        self.center.bind("<Configure>", self._update_scroll_region)
        self.content_canvas.bind("<Configure>", self._resize_scroll_window)
        self.content_canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        self.content_canvas.bind_all("<Button-4>", self._on_mousewheel)
        self.content_canvas.bind_all("<Button-5>", self._on_mousewheel)

        self.notice = tk.Label(self.center, text="", anchor="w", padx=12, pady=8)

        self.teleop_panel = tk.Frame(self.center, bd=0, highlightthickness=1)
        self.teleop_panel.grid(row=1, column=0, sticky="ew", pady=(4, 14))
        self.teleop_panel.columnconfigure(0, weight=1)

        self.welcome = tk.Frame(self.teleop_panel, bd=0, highlightthickness=0)
        self.welcome.grid(row=0, column=0, sticky="ew", padx=14, pady=(12, 14))
        self.welcome.columnconfigure(0, weight=1)
        self.welcome_title = tk.Label(self.welcome, text="准备开始", anchor="w")
        self.welcome_title.grid(row=0, column=0, sticky="w")
        self.welcome_text = tk.Label(
            self.welcome,
            text="配置连接参数后启动程序。对齐完成后可发送 Home 或开启遥操。",
            anchor="w",
            justify=tk.LEFT,
        )
        self.welcome_text.grid(row=1, column=0, sticky="ew", pady=(6, 12))

        self.quick_actions = tk.Frame(self.welcome, bd=0, highlightthickness=0)
        self.quick_actions.grid(row=2, column=0, sticky="w")
        self.start_button = ttk.Button(self.quick_actions, text="启动程序", command=self._start_process)
        self.start_button.grid(row=0, column=0, padx=(0, 8))
        self.home_button = ttk.Button(self.quick_actions, text="回 Home", command=lambda: self._tap_key("s"))
        self.home_button.grid(row=0, column=1, padx=(0, 8))
        self.teleop_button = ttk.Button(self.quick_actions, text="开启遥操", command=lambda: self._tap_key("d"))
        self.teleop_button.grid(row=0, column=2, padx=(0, 8))
        self.stop_button = ttk.Button(self.quick_actions, text="退出程序", command=self._stop_process)
        self.stop_button.grid(row=0, column=3, padx=(0, 8))

        self.config_panel = tk.Frame(self.teleop_panel, bd=0, highlightthickness=1)
        self.config_panel.grid(row=1, column=0, sticky="ew", padx=14, pady=(0, 14))
        for col in range(2):
            self.config_panel.columnconfigure(col, weight=1 if col == 1 else 0)

        self.ip_label = tk.Label(self.config_panel, text="机器人 IP", anchor="w")
        self.ip_label.grid(row=0, column=0, sticky="w", padx=(16, 8), pady=(14, 6))
        self.robot_ip_entry = ttk.Entry(self.config_panel, textvariable=self.robot_ip_var, width=18)
        self.robot_ip_entry.grid(row=0, column=1, sticky="ew", pady=(14, 6), padx=(0, 16))
        self.port_label = tk.Label(self.config_panel, text="编码器串口", anchor="w")
        self.port_label.grid(row=1, column=0, sticky="w", padx=(16, 8), pady=6)
        self.port_combo = ttk.Combobox(self.config_panel, textvariable=self.port_var, width=14)
        self.port_combo.grid(row=1, column=1, sticky="ew", pady=6, padx=(0, 16))
        self.baud_label = tk.Label(self.config_panel, text="波特率", anchor="w")
        self.baud_label.grid(row=2, column=0, sticky="w", padx=(16, 8), pady=(6, 14))
        self.baud_entry = ttk.Entry(self.config_panel, textvariable=self.baud_var, width=12)
        self.baud_entry.grid(row=2, column=1, sticky="ew", pady=(6, 14), padx=(0, 16))

        self.alignment_panel = tk.Frame(self.teleop_panel, bd=0, highlightthickness=1)
        self.alignment_panel.grid(row=2, column=0, sticky="ew", padx=14, pady=(0, 14))
        self._build_alignment_panel(self.alignment_panel)

        self.camera_panel = tk.Frame(self.center, bd=0, highlightthickness=1)
        self.camera_panel.grid(row=2, column=0, sticky="ew", pady=(0, 18))
        self._build_camera_panel(self.camera_panel)

        self.log_panel = tk.Frame(self.center, bd=0, highlightthickness=1)
        self.log_panel.grid(row=3, column=0, sticky="ew", pady=(0, 18))
        self._build_log_panel(self.log_panel)

    def _build_alignment_panel(self, parent):
        parent.columnconfigure(0, weight=1)
        head = tk.Frame(parent, bd=0, highlightthickness=0)
        head.grid(row=0, column=0, sticky="ew", padx=14, pady=(12, 8))
        head.columnconfigure(1, weight=1)
        self.alignment_title = tk.Label(head, text="对齐状态", anchor="w")
        self.alignment_title.grid(row=0, column=0, sticky="w", padx=(0, 12))
        self.alignment_summary = tk.Label(head, textvariable=self.alignment_summary_var, anchor="w")
        self.alignment_summary.grid(row=0, column=1, sticky="ew")

        grid = tk.Frame(parent, bd=0, highlightthickness=0)
        grid.grid(row=1, column=0, sticky="ew", padx=14, pady=(0, 14))
        for col in range(8):
            grid.columnconfigure(col, weight=1 if col > 0 else 0, uniform="joint")

        self.arm_a_label = tk.Label(grid, text="L/A", anchor="w")
        self.arm_a_label.grid(row=0, column=0, sticky="w", padx=(0, 8), pady=3)
        self.arm_b_label = tk.Label(grid, text="R/B", anchor="w")
        self.arm_b_label.grid(row=1, column=0, sticky="w", padx=(0, 8), pady=3)

        for arm, row in (("A", 0), ("B", 1)):
            for joint in range(7):
                cell = tk.Label(
                    grid,
                    text=f"{arm}{joint + 1}\n--",
                    font=("Microsoft YaHei UI", 9),
                    padx=7,
                    pady=6,
                    width=9,
                    borderwidth=0,
                    anchor="center",
                    justify=tk.CENTER,
                )
                cell.grid(row=row, column=joint + 1, sticky="ew", padx=3, pady=3)
                self.alignment_cells[(arm, joint)] = cell

    def _build_camera_panel(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(3, weight=1, minsize=CAMERA_PREVIEW_MIN_HEIGHT)

        head = tk.Frame(parent, bd=0, highlightthickness=0)
        head.grid(row=0, column=0, sticky="ew", padx=14, pady=(12, 8))
        head.columnconfigure(1, weight=1)
        self.camera_title = tk.Label(head, text="相机采集", anchor="w")
        self.camera_title.grid(row=0, column=0, sticky="w", padx=(0, 12))
        self.camera_status = tk.Label(head, textvariable=self.camera_status_var, anchor="w")
        self.camera_status.grid(row=0, column=1, sticky="ew")

        controls = tk.Frame(parent, bd=0, highlightthickness=0)
        controls.grid(row=1, column=0, sticky="ew", padx=14, pady=(0, 8))
        controls.columnconfigure(5, weight=1)
        self.camera_start_button = ttk.Button(controls, text="启动相机", command=self._start_cameras)
        self.camera_start_button.grid(row=0, column=0, padx=(0, 8))
        self.camera_stop_button = ttk.Button(controls, text="停止", command=self._stop_cameras, state=tk.DISABLED)
        self.camera_stop_button.grid(row=0, column=1, padx=(0, 8))
        self.camera_capture_button = ttk.Button(controls, text="采集一帧", command=self._capture_camera_frame, state=tk.DISABLED)
        self.camera_capture_button.grid(row=0, column=2, padx=(0, 8))
        self.camera_swap_button = ttk.Button(controls, text="交换左右腕", command=self._swap_wrist_cameras, state=tk.DISABLED)
        self.camera_swap_button.grid(row=0, column=3, padx=(0, 8))
        self.camera_dir_button = ttk.Button(controls, text="保存目录", command=self._select_camera_dir)
        self.camera_dir_button.grid(row=0, column=4, padx=(0, 8))
        self.camera_dir_entry = ttk.Entry(controls, textvariable=self.camera_dir_var)
        self.camera_dir_entry.grid(row=0, column=5, sticky="ew")

        self.camera_info = tk.Label(parent, textvariable=self.camera_info_var, anchor="w")
        self.camera_info.grid(row=2, column=0, sticky="ew", padx=14, pady=(0, 8))

        self.camera_view_frame = tk.Frame(parent, height=CAMERA_PREVIEW_MIN_HEIGHT, bd=0, highlightthickness=0)
        self.camera_view_frame.grid(row=3, column=0, sticky="ew", padx=14, pady=(0, 14))
        self.camera_view_frame.grid_propagate(False)
        self.camera_view_frame.columnconfigure(0, weight=1)
        self.camera_view_frame.rowconfigure(0, weight=1)

        self.camera_view = tk.Label(
            self.camera_view_frame,
            text="Head D435 | Left D405 | Right D405",
            anchor="center",
            justify=tk.CENTER,
            borderwidth=0,
        )
        self.camera_view.grid(row=0, column=0, sticky="nsew")

    def _build_log_panel(self, parent):
        parent.columnconfigure(0, weight=1)
        head = tk.Frame(parent, bd=0, highlightthickness=0)
        head.grid(row=0, column=0, sticky="ew", padx=14, pady=(12, 8))
        head.columnconfigure(1, weight=1)
        self.log_title = tk.Label(head, text="运行日志", anchor="w")
        self.log_title.grid(row=0, column=0, sticky="w", padx=(0, 12))
        self.log_hint = tk.Label(head, text="控制脚本输出与界面操作记录", anchor="w")
        self.log_hint.grid(row=0, column=1, sticky="ew")

        body = tk.Frame(parent, bd=0, highlightthickness=0)
        body.grid(row=1, column=0, sticky="ew", padx=14, pady=(0, 14))
        body.columnconfigure(0, weight=1)

        self.log_text = tk.Text(body, height=9, wrap="word", bd=0, highlightthickness=0, state=tk.DISABLED)
        self.log_text.grid(row=0, column=0, sticky="ew")
        log_scroll = ttk.Scrollbar(body, orient=tk.VERTICAL, command=self.log_text.yview)
        log_scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=log_scroll.set)

    def _apply_theme(self):
        palette = self._palette()
        self.configure(bg=palette["bg"])
        self.content_canvas.configure(bg=palette["bg"])
        self.style.configure("TButton", font=("Microsoft YaHei UI", 10), padding=(12, 7))
        self.style.map(
            "TButton",
            background=[("active", palette["surface_alt"]), ("disabled", palette["surface_alt"])],
            foreground=[("disabled", palette["muted"])],
        )
        self.style.configure(
            "TEntry",
            fieldbackground=palette["input"],
            foreground=palette["text"],
            bordercolor=palette["border"],
            lightcolor=palette["border"],
            darkcolor=palette["border"],
            padding=(8, 6),
        )
        self.style.configure(
            "TCombobox",
            fieldbackground=palette["input"],
            foreground=palette["text"],
            bordercolor=palette["border"],
            lightcolor=palette["border"],
            darkcolor=palette["border"],
            padding=(8, 6),
        )

        for widget in (self.shell, self.main, self.topbar, self.center):
            widget.configure(bg=palette["bg"])
        for widget in (self.sidebar, self.sidebar_head, self.sidebar_bottom):
            widget.configure(bg=palette["surface"], highlightbackground=palette["border"])
        for widget in (self.teleop_panel, self.config_panel, self.alignment_panel, self.camera_panel, self.log_panel, self.welcome, self.quick_actions):
            widget.configure(bg=palette["surface"], highlightbackground=palette["border"], highlightcolor=palette["accent"])

        self.logo_mark.configure(bg=palette["accent_bg"], fg=palette["accent"], font=("Microsoft YaHei UI", 14, "bold"))
        self.logo_title.configure(bg=palette["surface"], fg=palette["text"], font=("Microsoft YaHei UI", 11, "bold"))
        self.page_title.configure(bg=palette["bg"], fg=palette["text"], font=("Microsoft YaHei UI", 16, "bold"))
        self.page_subtitle.configure(bg=palette["bg"], fg=palette["muted"], font=("Microsoft YaHei UI", 9))
        self.status_pill.configure(bg=palette["accent_bg"], fg=palette["accent"], font=("Microsoft YaHei UI", 9))
        self.pid_label.configure(bg=palette["bg"], fg=palette["muted"], font=("Microsoft YaHei UI", 9))
        self.notice.configure(bg=palette["surface_alt"], fg=palette["danger"], font=("Microsoft YaHei UI", 9))
        self.welcome_title.configure(bg=palette["surface"], fg=palette["text"], font=("Microsoft YaHei UI", 18, "bold"))
        self.welcome_text.configure(bg=palette["surface"], fg=palette["muted"], font=("Microsoft YaHei UI", 10))

        for label in (self.ip_label, self.port_label, self.baud_label):
            label.configure(bg=palette["surface"], fg=palette["muted"], font=("Microsoft YaHei UI", 9))

        self._style_panel_labels(palette)
        self._reset_unset_alignment_cells()
        self.theme_button.configure(text="浅色模式" if self.theme_var.get() == "dark" else "深色模式")
        self._update_sidebar()

    def _style_panel_labels(self, palette):
        panel_labels = (
            self.alignment_title,
            self.alignment_summary,
            self.arm_a_label,
            self.arm_b_label,
            self.camera_title,
            self.camera_status,
            self.camera_info,
            self.camera_view,
            self.log_title,
            self.log_hint,
        )
        for label in panel_labels:
            label.configure(bg=palette["surface"], fg=palette["text"])
        self.alignment_title.configure(font=("Microsoft YaHei UI", 11, "bold"))
        self.camera_title.configure(font=("Microsoft YaHei UI", 11, "bold"))
        self.log_title.configure(font=("Microsoft YaHei UI", 11, "bold"))
        self.log_hint.configure(fg=palette["muted"], font=("Microsoft YaHei UI", 9))
        self.camera_view_frame.configure(bg=palette["surface_alt"])
        self.camera_view.configure(bg=palette["surface_alt"], fg=palette["muted"], font=("Microsoft YaHei UI", 10))
        for label in (self.alignment_summary, self.arm_a_label, self.arm_b_label, self.camera_status, self.camera_info):
            label.configure(fg=palette["muted"], font=("Microsoft YaHei UI", 9))
        for panel in (self.alignment_panel, self.camera_panel, self.log_panel):
            for child in panel.winfo_children():
                if isinstance(child, tk.Frame):
                    child.configure(bg=palette["surface"])
        self.log_text.configure(
            bg=palette["surface_alt"],
            fg=palette["text"],
            insertbackground=palette["text"],
            selectbackground=palette["accent_bg"],
            selectforeground=palette["text"],
            font=("Consolas", 9),
        )
        self.camera_view_frame.configure(bg=palette["surface_alt"])

    def _layout_center(self, _event=None):
        width = self.content_canvas.winfo_width()
        if width <= 1:
            return
        padding = max(14, int((width - 1240) / 2))
        available_width = max(1, width - padding * 2)
        self.content_canvas.coords(self.center_window, padding, 0)
        self.content_canvas.itemconfigure(self.center_window, width=available_width)
        self._update_scroll_region()

    def _resize_scroll_window(self, _event=None):
        self._layout_center()

    def _update_scroll_region(self, _event=None):
        if hasattr(self, "content_canvas"):
            self.content_canvas.configure(scrollregion=self.content_canvas.bbox("all"))

    def _on_mousewheel(self, event):
        if not hasattr(self, "content_canvas"):
            return None
        if getattr(event, "num", None) == 4:
            self.content_canvas.yview_scroll(-3, "units")
        elif getattr(event, "num", None) == 5:
            self.content_canvas.yview_scroll(3, "units")
        else:
            steps = int(-1 * (event.delta / 120))
            if steps:
                self.content_canvas.yview_scroll(steps, "units")
        return "break"

    def _toggle_sidebar(self):
        self.sidebar_collapsed = not self.sidebar_collapsed
        self.sidebar.configure(width=72 if self.sidebar_collapsed else 248)
        self._update_sidebar()

    def _update_sidebar(self):
        if self.sidebar_collapsed:
            self.logo_title.grid_remove()
            self.settings_button.configure(text="S")
            self.theme_button.configure(text="L" if self.theme_var.get() == "dark" else "D")
            self.sidebar_toggle.configure(text=">")
        else:
            self.logo_title.grid()
            self.settings_button.configure(text="设置")
            self.theme_button.configure(text="浅色模式" if self.theme_var.get() == "dark" else "深色模式")
            self.sidebar_toggle.configure(text="<")

    def _focus_settings(self):
        self.robot_ip_entry.focus_set()
        self._hide_notice()

    def _toggle_theme(self):
        self.theme_var.set("dark" if self.theme_var.get() == "light" else "light")
        self._apply_theme()

    def _show_notice(self, text, is_error=True):
        palette = self._palette()
        self.notice.configure(text=text, fg=palette["danger"] if is_error else palette["muted"])
        self.notice.grid(row=0, column=0, sticky="ew", pady=(0, 12))

    def _hide_notice(self):
        self.notice.grid_remove()

    def _set_busy(self, busy):
        self.busy = busy
        self._refresh_button_states()

    def _start_cameras(self):
        if self.camera_running:
            return
        try:
            from PIL import Image, ImageTk
        except Exception as exc:
            self._show_notice(f"缺少 Pillow，无法显示/保存相机画面: {exc}")
            self._append_log(f"[相机] 缺少 Pillow: {exc}\n")
            return

        self.pil_image = Image
        self.pil_image_tk = ImageTk
        self.camera_status_var.set("正在启动相机...")
        self.camera_start_button.configure(state=tk.DISABLED)
        self.camera_stop_button.configure(state=tk.DISABLED)
        self.camera_capture_button.configure(state=tk.DISABLED)
        self.camera_swap_button.configure(state=tk.DISABLED)
        self.update_idletasks()

        hub = RealSenseCameraHub(width=640, height=480, fps=30)
        starter = threading.Thread(target=self._camera_start_worker, args=(hub,), daemon=True)
        starter.start()

    def _camera_start_worker(self, hub):
        try:
            meta = hub.start()
        except Exception as exc:
            try:
                hub.stop()
            except Exception:
                pass
            self.after(0, self._camera_start_failed, str(exc))
            return
        self.after(0, self._camera_start_succeeded, hub, meta)

    def _camera_start_failed(self, error):
        self.camera_status_var.set("相机启动失败")
        self.camera_start_button.configure(state=tk.NORMAL)
        self.camera_stop_button.configure(state=tk.DISABLED)
        self.camera_capture_button.configure(state=tk.DISABLED)
        self.camera_swap_button.configure(state=tk.DISABLED)
        self.camera_view.configure(image="", text=f"相机启动失败\n{error}")
        self._show_notice(f"相机启动失败: {error}")
        self._append_log(f"[相机] 启动失败: {error}\n")

    def _camera_start_succeeded(self, hub, meta):
        self.camera_hub = hub
        self.camera_latest_meta = meta
        with self.camera_frames_lock:
            self.camera_latest_frames = {
                role: frame.copy()
                for role, frame in hub.latest_frames.items()
                if frame is not None
            }
        self.camera_stop_event = threading.Event()
        self.camera_running = True
        self.camera_thread = threading.Thread(target=self._camera_read_loop, daemon=True)
        self.camera_thread.start()
        self.camera_status_var.set("相机运行中")
        self.camera_info_var.set(self._format_camera_info(meta))
        self.camera_stop_button.configure(state=tk.NORMAL)
        self.camera_capture_button.configure(state=tk.NORMAL)
        self.camera_swap_button.configure(state=tk.NORMAL if "left_wrist" in meta and "right_wrist" in meta else tk.DISABLED)
        self._append_log(f"[相机] 已启动: {self.camera_info_var.get()}\n")
        self._update_camera_preview()

    def _stop_cameras(self, quiet=False):
        if not self.camera_running and self.camera_hub is None:
            return
        self.camera_running = False
        self.camera_stop_event.set()
        if self.camera_thread and self.camera_thread.is_alive() and threading.current_thread() is not self.camera_thread:
            self.camera_thread.join(timeout=1.5)
        if self.camera_hub is not None:
            self.camera_hub.stop()
        self.camera_hub = None
        self.camera_thread = None
        with self.camera_frames_lock:
            self.camera_latest_frames = {}
        self.camera_photo = None
        self.camera_status_var.set("相机已停止")
        self.camera_info_var.set("等待启动相机")
        self.camera_view.configure(image="", text="Head D435 | Left D405 | Right D405")
        self.camera_start_button.configure(state=tk.NORMAL)
        self.camera_stop_button.configure(state=tk.DISABLED)
        self.camera_capture_button.configure(state=tk.DISABLED)
        self.camera_swap_button.configure(state=tk.DISABLED)
        if not quiet:
            self._append_log("[相机] 已停止\n")

    def _camera_read_loop(self):
        while not self.camera_stop_event.is_set():
            try:
                frames = self.camera_hub.read_frames() if self.camera_hub else {}
            except Exception as exc:
                self.log_queue.put(f"[相机] 读取失败: {exc}\n")
                break
            if frames:
                with self.camera_frames_lock:
                    self.camera_latest_frames = frames
            time.sleep(0.01)

    def _update_camera_preview(self):
        if not self.camera_running:
            return
        if self.camera_thread and not self.camera_thread.is_alive():
            self._stop_cameras(quiet=True)
            self._show_notice("相机线程已停止，请检查 RealSense 连接")
            return
        with self.camera_frames_lock:
            frames = {role: frame.copy() for role, frame in self.camera_latest_frames.items()}
        if frames:
            photo = self._frames_to_photo(frames)
            self.camera_photo = photo
            self.camera_view.configure(image=photo, text="")
        self.after(33, self._update_camera_preview)

    def _frames_to_photo(self, frames):
        mosaic = self._build_camera_mosaic(frames, include_labels=True)
        view_width = self.camera_view_frame.winfo_width()
        view_height = self.camera_view_frame.winfo_height()
        if view_width <= 20:
            view_width = 900
        if view_height <= 20:
            view_height = CAMERA_PREVIEW_MIN_HEIGHT
        max_width = max(320, view_width - 8)
        max_height = max(260, view_height - 8)
        scale = min(max_width / mosaic.width, max_height / mosaic.height)
        target_size = (max(1, int(mosaic.width * scale)), max(1, int(mosaic.height * scale)))
        resample = getattr(getattr(self.pil_image, "Resampling", self.pil_image), "BILINEAR")
        resized = mosaic.resize(target_size, resample)
        return self.pil_image_tk.PhotoImage(resized)

    def _build_camera_mosaic(self, frames, include_labels=True):
        from PIL import ImageDraw

        available = [frame for frame in frames.values() if frame is not None]
        if available:
            tile_h, tile_w = available[0].shape[:2]
        else:
            tile_w, tile_h = 640, 480
        label_h = 28 if include_labels else 0
        row_h = tile_h + label_h
        mosaic_w = tile_w * 2
        mosaic_h = row_h * 2
        mosaic = self.pil_image.new("RGB", (mosaic_w, mosaic_h), (18, 20, 22))
        draw = ImageDraw.Draw(mosaic)

        def make_tile(role):
            frame = frames.get(role)
            if frame is None:
                return self.pil_image.new("RGB", (tile_w, tile_h), (36, 40, 44))
            tile = self.pil_image.fromarray(frame)
            if tile.size != (tile_w, tile_h):
                resample = getattr(getattr(self.pil_image, "Resampling", self.pil_image), "BILINEAR")
                tile = tile.resize((tile_w, tile_h), resample)
            return tile

        def paste_camera(role, label, x, y, cell_w):
            if include_labels:
                draw.rectangle((x, y, x + cell_w, y + label_h), fill=(24, 28, 32))
                draw.text((x + 10, y + 7), label, fill=(240, 244, 248))
            tile = make_tile(role)
            tile_x = x + max(0, (cell_w - tile_w) // 2)
            mosaic.paste(tile, (tile_x, y + label_h))

        paste_camera("head", "头部 D435", 0, 0, mosaic_w)
        paste_camera("left_wrist", "左腕 D405", 0, row_h, tile_w)
        paste_camera("right_wrist", "右腕 D405", tile_w, row_h, tile_w)
        return mosaic

    def _capture_camera_frame(self):
        with self.camera_frames_lock:
            frames = {role: frame.copy() for role, frame in self.camera_latest_frames.items()}
        if not frames:
            self._show_notice("还没有收到相机画面", is_error=False)
            return

        self.camera_capture_count += 1
        stamp = time.strftime("%Y%m%d_%H%M%S")
        sample_dir = os.path.join(self.camera_dir_var.get().strip() or CAMERA_DEFAULT_DIR, f"{stamp}_{self.camera_capture_count:04d}")
        try:
            os.makedirs(sample_dir, exist_ok=True)
            for role, frame in frames.items():
                self.pil_image.fromarray(frame).save(os.path.join(sample_dir, f"{role}.jpg"), quality=95)
            self._build_camera_mosaic(frames, include_labels=True).save(os.path.join(sample_dir, "mosaic.jpg"), quality=95)
            metadata = {
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "roles": {
                    role: {
                        "label": label,
                        "serial": self.camera_latest_meta.get(role, {}).get("serial"),
                        "name": self.camera_latest_meta.get(role, {}).get("name"),
                        "size": self.camera_latest_meta.get(role, {}).get("size"),
                        "file": f"{role}.jpg" if role in frames else None,
                    }
                    for role, label in CAMERA_ROLES
                },
                "mosaic": "mosaic.jpg",
            }
            with open(os.path.join(sample_dir, "metadata.json"), "w", encoding="utf-8") as fp:
                json.dump(metadata, fp, ensure_ascii=False, indent=2)
        except Exception as exc:
            self._show_notice(f"采集保存失败: {exc}")
            self._append_log(f"[相机] 采集保存失败: {exc}\n")
            return

        self.camera_status_var.set(f"已采集 {self.camera_capture_count} 帧")
        self._append_log(f"[相机] 已保存采集帧: {sample_dir}\n")

    def _select_camera_dir(self):
        selected = filedialog.askdirectory(initialdir=self.camera_dir_var.get() or SCRIPT_DIR)
        if selected:
            self.camera_dir_var.set(selected)
            self._append_log(f"[相机] 保存目录: {selected}\n")

    def _swap_wrist_cameras(self):
        if self.camera_hub is None or not self.camera_hub.swap_wrist_roles():
            self._show_notice("当前没有左右腕两路相机可交换", is_error=False)
            return
        with self.camera_frames_lock:
            left = self.camera_latest_frames.get("left_wrist")
            right = self.camera_latest_frames.get("right_wrist")
            if right is not None:
                self.camera_latest_frames["left_wrist"] = right
            else:
                self.camera_latest_frames.pop("left_wrist", None)
            if left is not None:
                self.camera_latest_frames["right_wrist"] = left
            else:
                self.camera_latest_frames.pop("right_wrist", None)
        self.camera_latest_meta = dict(self.camera_hub.pipelines)
        self.camera_info_var.set(self._format_camera_info(self.camera_latest_meta))
        self._append_log("[相机] 已交换左右腕相机\n")

    def _format_camera_info(self, meta):
        parts = []
        for role, label in CAMERA_ROLES:
            item = meta.get(role)
            if item:
                width, height, fps = item.get("size", ("?", "?", "?"))
                color_format = item.get("format")
                format_suffix = f" {color_format}" if color_format else ""
                parts.append(f"{label}: {item.get('name', 'RealSense')} #{item.get('serial', '-')} {width}x{height}@{fps}{format_suffix}")
            else:
                parts.append(f"{label}: 未连接")
        return " | ".join(parts)

    def _refresh_ports(self):
        try:
            import serial.tools.list_ports
            ports = [port.device for port in serial.tools.list_ports.comports()]
            self._hide_notice()
        except Exception as exc:
            ports = []
            if hasattr(self, "notice"):
                self._show_notice(f"串口列表读取失败: {exc}")
            self._append_log(f"[界面] 串口列表读取失败: {exc}\n")

        if self.port_var.get() and self.port_var.get() not in ports:
            ports.insert(0, self.port_var.get())
        self.port_combo["values"] = ports

    def _start_process(self):
        if self._is_running():
            self._append_log("[界面] 程序已在运行。\n")
            return

        if not FROZEN and not os.path.isfile(CONTROL_SCRIPT):
            messagebox.showerror("启动失败", f"找不到控制脚本:\n{CONTROL_SCRIPT}")
            self._show_notice("启动失败：找不到控制脚本。")
            return

        baud = self.baud_var.get().strip()
        if not baud.isdigit():
            messagebox.showerror("参数错误", "波特率必须是数字")
            self._show_notice("参数错误：波特率必须是数字。")
            return

        env = os.environ.copy()
        env["PYTHONIOENCODING"] = "utf-8"
        env["PYTHONUNBUFFERED"] = "1"
        env["GUJIAYAOCAO_ROBOT_IP"] = self.robot_ip_var.get().strip()
        env["GUJIAYAOCAO_ENCODER_PORT"] = self.port_var.get().strip()
        env["GUJIAYAOCAO_ENCODER_BAUDRATE"] = baud

        creationflags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0

        # 打包后由同一个 exe 以 --run-control 模式运行控制脚本
        command = [sys.executable, "--run-control"] if FROZEN else [sys.executable, "-u", CONTROL_SCRIPT]

        self.status_var.set("启动中")
        self._set_busy(True)
        self._reset_alignment_panel()
        try:
            self.process = subprocess.Popen(
                command,
                cwd=SCRIPT_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                text=True,
                encoding="utf-8",
                errors="replace",
                env=env,
                creationflags=creationflags,
            )
        except Exception as exc:
            self.process = None
            self._set_busy(False)
            messagebox.showerror("启动失败", str(exc))
            self._show_notice(f"启动失败: {exc}")
            return

        self.reader_thread = threading.Thread(target=self._read_process_output, daemon=True)
        self.reader_thread.start()
        self._append_log(f"[界面] 已启动: PID={self.process.pid}\n")
        self._set_busy(False)
        self._refresh_state()

    def _read_process_output(self):
        if not self.process or not self.process.stdout:
            return
        for line in self.process.stdout:
            self.log_queue.put(line)
        code = self.process.wait()
        self.log_queue.put(f"[界面] 程序已退出，退出码: {code}\n")

    def _stop_process(self):
        if not self._is_running():
            self._append_log("[界面] 程序未运行。\n")
            return

        self.status_var.set("停止中")
        self._set_busy(True)
        self._append_log("[界面] 发送 ESC 退出指令。\n")
        self._tap_key("esc", quiet=True)
        self.after(1800, self._terminate_if_needed)

    def _terminate_if_needed(self):
        if not self._is_running():
            self._set_busy(False)
            self._refresh_state()
            return
        self._append_log("[界面] ESC 后仍在运行，执行进程终止。\n")
        try:
            if os.name == "nt":
                # 打包后的 onefile exe 是“引导进程 + 子进程”，需要杀掉整个进程树
                subprocess.run(
                    ["taskkill", "/F", "/T", "/PID", str(self.process.pid)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )
            else:
                self.process.terminate()
        except Exception as exc:
            self._append_log(f"[界面] 终止失败: {exc}\n")
            self._show_notice(f"终止失败: {exc}")
        self._set_busy(False)
        self._refresh_state()

    def _tap_key(self, key_name, quiet=False):
        if key_name != "esc" and not self._is_running():
            self._append_log("[界面] 程序未运行，按键未发送。\n")
            self._show_notice("程序未运行，按键未发送。", is_error=False)
            return
        if self.keyboard is None:
            messagebox.showerror("按键失败", "未能加载 pynput，无法发送控制按键")
            self._show_notice("未能加载 pynput，无法发送控制按键。")
            return

        try:
            if key_name == "esc":
                self.keyboard.press(Key.esc)
                time.sleep(0.05)
                self.keyboard.release(Key.esc)
            else:
                self.keyboard.press(key_name)
                time.sleep(0.05)
                self.keyboard.release(key_name)
            if not quiet:
                self._append_log(f"[界面] 已发送按键: {key_name}\n")
            self._hide_notice()
        except Exception as exc:
            self._append_log(f"[界面] 发送按键失败: {exc}\n")
            self._show_notice(f"发送按键失败: {exc}")

    def _open_script_dir(self):
        try:
            if os.name == "nt":
                os.startfile(SCRIPT_DIR)
            else:
                subprocess.Popen(["xdg-open", SCRIPT_DIR])
        except Exception as exc:
            self._append_log(f"[界面] 打开目录失败: {exc}\n")
            self._show_notice(f"打开目录失败: {exc}")

    def _is_running(self):
        return self.process is not None and self.process.poll() is None

    def _refresh_state(self):
        if self._is_running():
            self.status_var.set("运行中")
            self.pid_var.set(f"PID {self.process.pid}")
        elif not self.busy:
            self.status_var.set("未启动")
            self.pid_var.set("PID -")
        self._refresh_button_states()
        self.after(1000, self._refresh_state)

    def _refresh_button_states(self):
        running = self._is_running()
        self.start_button.configure(state=tk.DISABLED if self.busy or running else tk.NORMAL)
        self.home_button.configure(state=tk.NORMAL if running and not self.busy else tk.DISABLED)
        self.teleop_button.configure(state=tk.NORMAL if running and not self.busy else tk.DISABLED)
        self.stop_button.configure(state=tk.NORMAL if running and not self.busy else tk.DISABLED)

    def _poll_log_queue(self):
        while True:
            try:
                line = self.log_queue.get_nowait()
            except queue.Empty:
                break
            if line.startswith("__ALIGN_STATUS__"):
                self._handle_alignment_status(line[len("__ALIGN_STATUS__"):])
                continue
            self._append_log(line)
        self.after(50, self._poll_log_queue)

    def _handle_alignment_status(self, payload_text):
        try:
            payload = json.loads(payload_text)
        except json.JSONDecodeError:
            return

        palette = self._palette()
        start_tol = float(payload.get("start_tol", 5.0))
        stage = payload.get("stage", "ready")
        ok_count = 0
        total_count = 14

        for arm in ("A", "B"):
            arm_data = payload.get(arm, {})
            start_err = arm_data.get("start_err") or [None] * 7
            skeleton_err = arm_data.get("skeleton_err") or [None] * 7
            skeleton_aligned = arm_data.get("skeleton_aligned") or [False] * 7

            for joint in range(7):
                start_value = start_err[joint] if joint < len(start_err) else None
                skeleton_value = skeleton_err[joint] if joint < len(skeleton_err) else None
                skeleton_latched = bool(skeleton_aligned[joint]) if joint < len(skeleton_aligned) else False
                start_ok = start_value is not None and start_value <= start_tol
                if stage == "arm":
                    aligned = start_ok
                elif stage == "skeleton":
                    aligned = skeleton_latched
                else:
                    aligned = start_ok and skeleton_latched
                if aligned:
                    ok_count += 1

                start_text = "--" if start_value is None else f"{start_value:.1f}°"
                skeleton_text = "--" if skeleton_value is None else f"{skeleton_value:.1f}°"
                cell = self.alignment_cells.get((arm, joint))
                if cell is not None:
                    cell.configure(
                        text=f"{arm}{joint + 1}\n起 {start_text}\n骨 {skeleton_text}",
                        bg=palette["success"] if aligned else palette["danger"],
                        fg="#ffffff",
                    )

        if stage == "arm":
            if ok_count == total_count:
                self.alignment_summary_var.set("第一步完成：机械臂已对齐，请继续对齐骨架")
            else:
                self.alignment_summary_var.set(f"第一步：机械臂已对齐 {ok_count}/{total_count} 个关节")
        elif stage == "skeleton":
            self.alignment_summary_var.set(f"第二步：骨架已对齐 {ok_count}/{total_count} 个关节")
        elif ok_count == total_count:
            self.alignment_summary_var.set("全部关节已对齐，可以启动摇操")
        else:
            self.alignment_summary_var.set(f"已对齐 {ok_count}/{total_count} 个关节；红色关节需要继续调整")

    def _reset_unset_alignment_cells(self):
        palette = self._palette()
        for cell in self.alignment_cells.values():
            if "--" in cell.cget("text"):
                cell.configure(bg=palette["surface_alt"], fg=palette["muted"])

    def _reset_alignment_panel(self):
        palette = self._palette()
        self.alignment_summary_var.set("未收到对齐数据")
        for (arm, joint), cell in self.alignment_cells.items():
            cell.configure(text=f"{arm}{joint + 1}\n--", bg=palette["surface_alt"], fg=palette["muted"])

    def _append_log(self, text):
        if not hasattr(self, "log_text"):
            return
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, text)
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _on_close(self):
        if self._is_running():
            if not messagebox.askyesno("退出界面", "控制程序仍在运行，是否先发送退出指令？"):
                return
            self._stop_cameras(quiet=True)
            self._stop_process()
            self.after(2200, self.destroy)
            return
        self._stop_cameras(quiet=True)
        self.destroy()


if __name__ == "__main__":
    app = TeleopFrontend()
    app.mainloop()