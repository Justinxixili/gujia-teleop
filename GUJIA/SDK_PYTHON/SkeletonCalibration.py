#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
纯Python版 STS编码器读取程序（Windows/Linux适配+语法修复）
无ROS2依赖，直接运行，控制台交互
功能：串口通信、数据解析、零点设置、方向设置、配置保存
"""
import serial
import serial.tools.list_ports
import time
import math
import configparser
import os
import sys
import threading
from typing import Optional, List, Dict
from dataclasses import dataclass
import signal

if os.name == "nt":
    import msvcrt
else:
    import select
    import termios
    import tty

# 全局退出标志（安全退出程序）
running = True

# 信号处理：Ctrl+C 退出
def signal_handler(sig, frame):
    global running  # 提前声明global
    print("\n\n正在退出程序...")
    running = False

signal.signal(signal.SIGINT, signal_handler)

@dataclass
class STSDriverConfig:
    timeout_ms: int = 500
    baudrate_map = {
        9600: 9600, 19200: 19200, 38400: 38400, 57600: 57600,
        115200: 115200, 230400: 230400, 460800: 460800, 500000: 500000,
        576000: 576000, 921600: 921600, 1000000: 1000000
    }

class STSDriver:
    """STS编码器串口驱动"""
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
        """计算校验和"""
        sum_val = sum(data)
        return sum_val & 0xFF

    def send_and_receive(self, cmd: List[int]) -> Optional[List[int]]:
        """发送指令并接收数据"""
        if not self.fd or not self.fd.is_open:
            return None

        with self.ser_mutex:
            self.fd.reset_input_buffer()
            try:
                self.fd.write(bytes(cmd))
                time.sleep(0.02)
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
                    time.sleep(0.05)
                    if no_data_count >= 2 and response:
                        break

        return response if response else None

class EncoderReader:
    """编码器核心处理类"""
    @staticmethod
    def default_port() -> str:
        env_port = os.environ.get("ENCODER_PORT")
        if env_port:
            return env_port
        if os.name == "nt":
            ports = list(serial.tools.list_ports.comports())
            return ports[0].device if ports else "COM3"

        preferred_ports = ("/dev/ttyUSB0", "/dev/ttyACM0")
        for port in preferred_ports:
            if os.path.exists(port):
                return port

        ports = list(serial.tools.list_ports.comports())
        if ports:
            return ports[0].device
        return "/dev/ttyUSB0"

    def __init__(self):
        # ===================== 可手动修改的配置 =====================
        self.port = self.default_port()  # Windows: COM3/COM4; Ubuntu: /dev/ttyUSB0 或 /dev/ttyACM0
        self.baudrate = 1000000     # 波特率
        self.send_frequency = 10.0  # 读取频率(Hz)
        self.filter_id = 0          # 0=显示所有编码器
        self.enable_checksum = True # 启用校验
        self.encoder_resolution = 4096  # 编码器分辨率s
        # self.config_file = "servo_config.ini" # 配置文件
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(script_dir, "servo_config.ini")
        # ==========================================================

        self.radian_scale = 2 * math.pi / self.encoder_resolution
        self.cmd = [0xFF,0xFF,0xFE,0x14,0x82,0x38,0x02,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0xA9]

        # 编码器状态缓存
        self.last_raw_position: Dict[int, int] = {}
        self.accumulated_position: Dict[int, int] = {}
        self.zero_offset_raw: Dict[int, int] = {}
        self.direction: Dict[int, int] = {}
        self.data_mutex = threading.Lock()
        self.console_mutex = threading.Lock()
        self.input_active = False

        # 初始化串口
        try:
            self.driver = STSDriver(self.port, self.baudrate)
        except Exception as e:
            print(f"错误: {e}")
            raise

        # 加载配置
        self.load_config()
        print("="*50)
        print("命令说明：")
        print("  按任意键进入命令输入，输入时实时刷屏会暂停")
        print("  set_zero 1    - 设置编码器1零点")
        print("  set_dir 1 -1  - 设置编码器1方向(1正向/-1反向)")
        print("  quit          - 退出程序")
        print("="*50)

    def load_config(self):
        """加载配置文件"""
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
        print(f"[配置] 已加载: {self.config_file}")

    def save_config(self):
        """保存配置文件"""
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
        """校验和验证"""
        if offset +7 >= len(data): return False
        s = sum(data[offset+2:offset+7]) & 0xFF
        return (~s & 0xFF) == data[offset+7]

    def get_accumulated_position(self, sid: int, raw: int) -> int:
        """计算累计位置（处理圈数溢出）"""
        with self.data_mutex:
            offset = self.zero_offset_raw.get(sid, 0)
            if sid not in self.accumulated_position:
                self.accumulated_position[sid] = raw - offset
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
        """计算弧度（含零点+方向）"""
        acc = self.get_accumulated_position(sid, raw)
        dir_val = self.direction.get(sid, 1)
        angle = math.atan2(math.sin(acc * dir_val * self.radian_scale), math.cos(acc * dir_val * self.radian_scale))
        return angle

    def set_zero(self, sid: int) -> bool:
        """设置零点"""
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
        """设置方向"""
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
        """解析并打印数据"""
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

        lines = []
        for i in range(len(data)//GROUP):
            off = i*GROUP
            sid = data[off+2]
            raw = (data[off+6] <<8) | data[off+5]
            rad = self.get_radian(sid, raw)
            deg = rad * 180 / math.pi

            if self.filter_id !=0 and sid != self.filter_id:
                continue
            lines.append(f"ID:{sid:2d} | 原始值:{raw:4d} | 弧度:{rad:7.3f} | 角度:{deg:6.1f}°")

        if self.input_active:
            return

        with self.console_mutex:
            if self.input_active:
                return
            self.clear_console()
            print("="*30 + " 编码器实时数据 " + "="*30)
            for line in lines:
                print(line)
            print("="*66)
            print("按任意键输入命令：set_zero 1 / set_dir 1 -1 / quit")

    def clear_console(self):
        """跨平台清屏。"""
        if os.name == "nt":
            os.system("cls")
        else:
            print("\033[2J\033[H", end="", flush=True)

    def _read_console_key(self, block: bool = False) -> Optional[str]:
        """跨平台读取单个按键；无按键时返回 None。"""
        if os.name == "nt":
            if not block and not msvcrt.kbhit():
                return None
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):
                msvcrt.getwch()
                return ""
            return ch

        if not sys.stdin.isatty():
            return None

        timeout = None if block else 0
        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        if not readable:
            return None
        return sys.stdin.read(1)

    def read_loop(self):
        """定时读取线程"""
        period = 1.0 / self.send_frequency
        while running:
            start = time.time()
            res = self.driver.send_and_receive(self.cmd)
            if res:
                self.parse_data(res)
            # 控制读取频率
            elapsed = time.time() - start
            sleep_time = max(0, period - elapsed)
            time.sleep(sleep_time)

    def read_command(self) -> str:
        """跨平台控制台逐字符读命令，避免实时刷屏覆盖输入。"""
        global running
        old_tty_attrs = None
        if os.name != "nt" and sys.stdin.isatty():
            old_tty_attrs = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())

        try:
            while running:
                ch = self._read_console_key(block=False)
                if ch is None:
                    time.sleep(0.05)
                    continue
                if ch == "":
                    continue

                self.input_active = True
                buf = []
                with self.console_mutex:
                    print("\n输入命令: ", end="", flush=True)
                while running:
                    if ch is None:
                        ch = self._read_console_key(block=True)
                    if ch is None:
                        continue
                    if ch in ("\r", "\n"):
                        with self.console_mutex:
                            print()
                        return "".join(buf).strip()
                    if ch == "\x03":
                        running = False
                        return "quit"
                    if ch in ("\x08", "\x7f"):
                        if buf:
                            buf.pop()
                            with self.console_mutex:
                                print("\b \b", end="", flush=True)
                        ch = None
                        continue
                    if ch == "":
                        ch = None
                        continue
                    buf.append(ch)
                    with self.console_mutex:
                        print(ch, end="", flush=True)
                    ch = None
            return "quit"
        finally:
            if old_tty_attrs is not None:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_tty_attrs)

    def cmd_loop(self):
        """控制台命令线程（修复global声明）"""
        global running  # 关键修复：在函数开头就声明global
        while running:
            try:
                cmd = self.read_command()
                if not running: break
                args = cmd.split()
                if not args:
                    self.input_active = False
                    continue

                if args[0] == "quit":
                    running = False  # 现在可以正常修改全局变量
                elif args[0] == "set_zero" and len(args)==2:
                    self.set_zero(int(args[1]))
                elif args[0] == "set_dir" and len(args)==3:
                    self.set_direction(int(args[1]), int(args[2]))
                else:
                    print("无效命令！支持：set_zero / set_dir / quit")
                if running:
                    print("2秒后恢复实时显示，按任意键可再次输入命令。")
                    time.sleep(2.0)
                self.input_active = False
            except Exception as e:
                print(f"命令执行错误: {e}")
                self.input_active = False
                continue

def main():
    try:
        reader = EncoderReader()
        # 启动双线程
        t1 = threading.Thread(target=reader.read_loop, daemon=True)
        t2 = threading.Thread(target=reader.cmd_loop, daemon=True)
        t1.start()
        t2.start()

        # 等待退出
        while running:
            time.sleep(0.1)
        reader.save_config()
        print("程序已正常退出")
    except Exception as e:
        print(f"程序启动失败: {e}")

if __name__ == "__main__":
    main()
