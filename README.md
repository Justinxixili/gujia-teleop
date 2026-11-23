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
   ├─ KM_gripper.py                  夹爪控制模块
   ├─ footkeyboard.py                脚踏键盘测试脚本（独立运行）
   ├─ servo_config.ini               编码器零位/方向配置
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

依赖：`pynput`、`numpy`、`pyserial`、`Pillow`、`pyrealsense2`（相机功能需要）。

控制脚本通过环境变量接收参数：

| 环境变量 | 说明 | 默认值 |
|---|---|---|
| `GUJIAYAOCAO_ROBOT_IP` | 机器人 IP | `192.168.1.190` |
| `GUJIAYAOCAO_ENCODER_PORT` | 编码器串口 | `/dev/ttyACM0` |
| `GUJIAYAOCAO_ENCODER_BAUDRATE` | 串口波特率 | `1000000` |

## 说明

- `libMarvinSDK` / `libKine` 为厂商提供的 SDK 动态库。
- `camera_dataset/` 下的采集结果不纳入版本管理。
