# 骨架摇操（gujia-teleop）

基于马文（Marvin）机械臂 SDK 的骨架遥操作（摇操）系统。

## 目录结构

```
GUJIA/
└─ SDK_PYTHON/
   ├─ gujiayaocao_0817_frontend.py   入口：Tkinter 图形控制台
   ├─ gujiayaocao_0817.py            控制脚本：机械臂对齐 / 遥操 / 回 Home
   ├─ fx_robot.py                    机器人控制 SDK 封装
   ├─ fx_kine.py                     运动学计算接口
   ├─ KM_CAN.py                      CAN 电机与夹爪控制
   ├─ CalibrationOfTheOrigin.py      舵机原点标定工具（图形界面，独立运行）
   ├─ footkeyboard.py                脚踏键盘测试脚本（独立运行）
   ├─ servo_config.ini               编码器零位/方向配置
   ├─ gripper_config.ini             夹爪张开/闭合端点映射
   ├─ teleop_config.ini              摇操参数配置
   ├─ requirements.txt               Python 依赖
   ├─ libMarvinSDK.dll / .so         机器人控制动态库
   ├─ libKine.dll / .so              运动学动态库
   ├─ start_gujiayaocao_frontend.bat Windows 启动脚本
   └─ camera_dataset/                相机采集输出目录
```

## 功能

- 启动/停止控制脚本，配置机器人 IP、编码器串口与波特率
- 通过按键触发原有 s（回 Home）/ d（开启遥操）/ ESC（退出）控制
- 实时显示 14 个关节（双臂 × 7）的对齐状态
- RealSense 三路相机（头部 D435/D455、左右腕 D405）拼接预览与按帧采集

## 运行

```bat
cd GUJIA\SDK_PYTHON
start_gujiayaocao_frontend.bat
```

或：

```bat
python gujiayaocao_0817_frontend.py
```

依赖：`pip install -r GUJIA/SDK_PYTHON/requirements.txt`（相机功能另需 `pyrealsense2`）。

控制脚本通过环境变量接收参数：

| 环境变量 | 说明 | 默认值 |
|---|---|---|
| `GUJIAYAOCAO_ROBOT_IP` | 机器人 IP | `192.168.1.190` |
| `GUJIAYAOCAO_ENCODER_PORT` | 编码器串口 | `/dev/ttyACM0` |
| `GUJIAYAOCAO_ENCODER_BAUDRATE` | 串口波特率 | `1000000` |

## 说明

- `libMarvinSDK` / `libKine` 为厂商提供的 SDK 动态库。
- `camera_dataset/` 下的采集结果不纳入版本管理。
- 编码器 ID 与关节的对应关系见 `servo_config.ini` 的 `[encoder_map]`，默认按 ID 1..16 顺序（A 臂 1–7、左夹爪 8、B 臂 9–15、右夹爪 16）；接线不同时按实际修改。

## 舵机原点标定

```bat
python CalibrationOfTheOrigin.py
```

流程：连接编码器串口 → 逐个关节转到机械正限位点「记正限位」→ 转到负限位点「记负限位」→ 程序自动计算原点 → 人工「确认」→ 「导出结果」生成独立的 ini 文件。

状态提示：记录限位后关节名称变蓝，正负限位都记录后变绿；每个记录按钮再点一次会清空该限位，可重新记录。

原点按编码器的实际运动路径中点计算，超过 180° 的关节也能正确标定。记录时请让关节从一个限位连续移动到另一个限位，中途不要绕圈；记录完成后，界面上的正/负限位角度以标定原点为零点显示，便于和额定关节范围对照。

注意：`servo_config.ini` **只读**，标定程序不会修改它；导出文件需人工核对后再合并。

## 夹爪映射

摇操中的夹爪不使用关节的中点原点，而是按 `gripper_config.ini` 里的张开/闭合原始值做线性映射：

```ini
[right]
closed_raw = 836   # 完全闭合时的编码器原始值
open_raw = 652     # 完全张开时的编码器原始值
target_closed = 1.1
target_open = -0.15
```

- 没有配置或两端相同的夹爪会退回旧的 `abs(角度)/9` 归一化；
- `servo_config.ini` 只读，夹爪端点单独放在这个文件里；
- 若张开/闭合方向相反，互换 `target_closed` 与 `target_open` 即可。

## 摇操参数配置（teleop_config.ini）

摇操的全部可调参数集中在 `teleop_config.ini`：

- `[robot]`：IP、起始/回 Home 点位、起始误差、骨架对齐误差、重新对齐阈值
- `[encoder]`：串口（留空自动）、波特率、读取频率、滤波系数、分辨率
- `[control]`：主循环周期、对齐状态刷新间隔、控制台打印间隔、插值步长、速度前馈频率
- `[gripper]`：夹爪使能、CAN ID、MIT 参数、机械限位、指令/反馈频率
- `[impedance]`：关节阻抗 K/D
- `[tool]`：工具负载偏移与动力学参数

环境变量 `GUJIAYAOCAO_*` 优先级高于配置文件（前端界面填写的 IP/串口/波特率会通过环境变量传入）。

对齐阶段主循环约 33Hz 检查，状态以 20Hz 推送到前端；**开始遥操后不再订阅/检查**，不会占用遥操通信。

## 便携版 exe（打包）

在 `SDK_PYTHON` 下执行：

```bat
python packaging/build_exe.py
```

生成 `release/`：

- `GujiaTeleop.exe` —— 摇操控制台（前端 + 控制脚本同一个 exe，控制进程由前端以 `--run-control` 启动）
- `OriginCalibration.exe` —— 舵机原点标定
- `servo_config.ini` / `gripper_config.ini` / `teleop_config.ini`
- `libMarvinSDK.dll` / `libKine.dll`
- `使用说明.txt`

同时生成 `SDK_PYTHON/GujiaTeleop_release.zip`，复制到其他 Windows 10/11 64 位电脑解压即可使用。打包产物不入库（见 `.gitignore`）。


