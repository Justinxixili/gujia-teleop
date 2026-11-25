# -*- coding: utf-8 -*-
"""构建两个便携版 exe 到 SDK_PYTHON/release。

用法（在 SDK_PYTHON 下）：
    python packaging/build_exe.py
"""
import os
import shutil
import subprocess
import sys

PACKAGING_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_DIR = os.path.dirname(PACKAGING_DIR)
PROJECT_ROOT = os.path.dirname(SDK_DIR)          # SDK_PYTHON 的父目录
RELEASE_DIR = os.path.join(SDK_DIR, "release")
BUILD_DIR = os.path.join(PACKAGING_DIR, "_build")
LAUNCHER = os.path.join(PACKAGING_DIR, "teleop_launcher.py")
CALIBRATION = os.path.join(SDK_DIR, "CalibrationOfTheOrigin.py")


def _run(cmd):
    print("> " + " ".join(f'"{c}"' if " " in c else c for c in cmd), flush=True)
    subprocess.check_call(cmd, cwd=PROJECT_ROOT)


def _pyinstaller(args):
    _run([sys.executable, "-m", "PyInstaller"] + args)


def build_teleop():
    _pyinstaller([
        "--noconfirm", "--clean", "--onefile", "--windowed",
        "--name", "GujiaTeleop",
        "--distpath", RELEASE_DIR,
        "--workpath", os.path.join(BUILD_DIR, "teleop"),
        "--specpath", BUILD_DIR,
        "--paths", SDK_DIR,
        "--paths", PROJECT_ROOT,
        "--hidden-import", "gujiayaocao_0817",
        "--hidden-import", "gujiayaocao_0817_frontend",
        "--hidden-import", "KM_CAN",
        "--hidden-import", "SDK_PYTHON.GentoRobot",
        "--hidden-import", "serial.tools.list_ports",
        "--hidden-import", "PIL.ImageTk",
        "--hidden-import", "PIL._tkinter_finder",
        "--collect-all", "pynput",
        "--collect-all", "pyrealsense2",
        "--add-binary", os.path.join(SDK_DIR, "libGentoSDKPY.dll") + ";.",
        LAUNCHER,
    ])


def build_calibration():
    _pyinstaller([
        "--noconfirm", "--clean", "--onefile", "--windowed",
        "--name", "OriginCalibration",
        "--distpath", RELEASE_DIR,
        "--workpath", os.path.join(BUILD_DIR, "calibration"),
        "--specpath", BUILD_DIR,
        "--hidden-import", "serial.tools.list_ports",
        CALIBRATION,
    ])


def copy_assets():
    for name in ("servo_config.ini", "gripper_config.ini", "teleop_config.ini"):
        shutil.copy2(os.path.join(SDK_DIR, name), os.path.join(RELEASE_DIR, name))
    for name in ("libGentoSDKPY.dll",):
        shutil.copy2(os.path.join(SDK_DIR, name), os.path.join(RELEASE_DIR, name))
    note = os.path.join(PACKAGING_DIR, "使用说明.txt")
    if os.path.isfile(note):
        shutil.copy2(note, os.path.join(RELEASE_DIR, "使用说明.txt"))


def make_zip():
    # 先删除 release 里旧的 zip，避免新 zip 把旧 zip 一起打进去
    for name in os.listdir(RELEASE_DIR):
        if name.lower().endswith(".zip"):
            os.remove(os.path.join(RELEASE_DIR, name))
    # 先在 SDK_PYTHON 下生成，再移入 release
    temp_base = os.path.join(SDK_DIR, "_release_tmp")
    temp_zip = temp_base + ".zip"
    if os.path.exists(temp_zip):
        os.remove(temp_zip)
    shutil.make_archive(temp_base, "zip", RELEASE_DIR)
    target = os.path.join(RELEASE_DIR, "GujiaTeleop_release.zip")
    shutil.move(temp_zip, target)
    print("zip: " + target)


def main():
    os.makedirs(RELEASE_DIR, exist_ok=True)
    os.makedirs(BUILD_DIR, exist_ok=True)
    build_teleop()
    build_calibration()
    copy_assets()
    make_zip()
    print("完成：", RELEASE_DIR)


if __name__ == "__main__":
    main()
